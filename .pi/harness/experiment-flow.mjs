import { closeSync, existsSync, mkdirSync, openSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { createHash } from "node:crypto";
import { arch, hostname, platform, release } from "node:os";
import { execFileSync, spawn } from "node:child_process";
import { inside, readJson, requireActive, updateProgress, writeJson } from "./research-context.mjs";

const RUN = /^R-\d{8}T\d{6}Z-[a-z0-9]+$/;
const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);
const sha = (data) => createHash("sha256").update(data).digest("hex");
const git = (root, args) => execFileSync("git", args, { cwd: root, encoding: "utf8", timeout: 5000 }).trim();
const positive = (n) => typeof n === "number" && Number.isFinite(n) && n > 0;
export const hostFingerprint = () => sha(JSON.stringify({ platform: platform(), arch: arch(), release: release(), hostname: hostname(), node: process.version }));

function contractTable(task, name, columns) {
  const rows = (task.sections[name] ?? "").split("\n").filter(line => line.trim().startsWith("|"))
    .map(line => line.trim().replace(/^\||\|$/g, "").split("|").map(cell => cell.trim()));
  const indexes = columns.map(column => rows[0]?.indexOf(column) ?? -1);
  if (indexes.includes(-1) || rows.length < 3 || rows[1].length !== rows[0].length || !rows[1].every(cell => /^:?-{3,}:?$/.test(cell)) ||
      rows.slice(2).some(row => row.length !== rows[0].length || indexes.some(index => !row[index]))) {
    throw new Error(`Incomplete ${name}`);
  }
}

function validateRequest(root, request, task) {
  if (request.version !== 1 || request.taskId !== task.taskId || request.stage !== task.stage) throw new Error("Request/TASK identity mismatch");
  if (task.robotRuns !== "0" || request.hardware !== "local-compute") throw new Error("Workflow only supports approved local compute, with Robot runs: 0");
  if (!request.environment || typeof request.environment !== "string") throw new Error("Declare the minimum environment description");
  const output = inside(root, request.outputDir), parts = request.outputDir.split("/");
  if (parts.length !== 3 || parts[0] !== "runs" || parts[1] !== task.taskId || !RUN.test(parts[2])) throw new Error("outputDir must be runs/<task-id>/<UTC Run ID>");
  if (!task.sections.Outputs.includes(`\`${request.outputDir}/\``) && !task.sections.Outputs.includes(`\`${request.outputDir}\``)) throw new Error("TASK Outputs must include the exact run directory");
  if (!positive(request.budget?.maxCostUnits) || !Number.isSafeInteger(request.budget?.maxWallTimeMs) || request.budget.maxWallTimeMs < 1) throw new Error("Declare positive cost and integer wall-time budgets");
  if (!Array.isArray(request.steps) || !request.steps.length || !Array.isArray(request.inputs)) throw new Error("Request needs steps and an explicit inputs list (may be empty in EXPLORE)");
  let previousCost = 0;
  const ids = new Set();
  for (const step of request.steps) {
    if (!/^[a-z][a-z0-9_-]*$/.test(step.id) || ["inputs", "review"].includes(step.id) || ids.has(step.id)) throw new Error("Step IDs must be unique simple names; inputs and review are reserved");
    ids.add(step.id);
    if (!["direct", "prerequisite"].includes(step.relation) || typeof step.purpose !== "string" || !step.purpose.trim()) throw new Error(`Step ${step.id} needs its relationship and purpose for the original question`);
    if (!positive(step.costUnits) || step.costUnits < previousCost) throw new Error("Steps must be ordered from cheap to expensive using declared costUnits");
    previousCost = step.costUnits;
    if (!Number.isSafeInteger(step.timeoutMs) || step.timeoutMs < 1 || step.timeoutMs > 2147483647 || step.timeoutMs > request.budget.maxWallTimeMs) throw new Error("Step timeout exceeds run budget or Node timer range");
    if (!Array.isArray(step.argv) || !step.argv.length || step.argv.some(arg => typeof arg !== "string" || !arg || arg.includes("\0"))) throw new Error("Use an explicit argv array; shell strings are not supported");
    if (!step.argv.some(arg => arg.includes("{output_dir}"))) throw new Error("Each command must receive explicit {output_dir}");
    if (!/^[\w.-]+\.json$/.test(step.resultFile ?? "") || step.resultFile === "execution.json") throw new Error("resultFile must be a simple JSON filename; execution.json is reserved");
    const rules = step.pass;
    if (!Array.isArray(rules) || !rules.length || rules.some(rule => !/^[a-zA-Z_][\w.]*$/.test(rule.field ?? "") ||
      !["eq", "lt", "lte", "gt", "gte"].includes(rule.op) || !["number", "boolean", "string"].includes(typeof rule.value) ||
      (typeof rule.value === "number" && !Number.isFinite(rule.value)) || (rule.op !== "eq" && typeof rule.value !== "number"))) {
      throw new Error("Declare deterministic pass rules; missing/non-numeric measurements never pass numeric rules");
    }
  }
  for (const input of request.inputs) {
    const file = inside(root, input);
    if (!statSync(file).isFile() || file === inside(root, "state/TASK.md") || file.startsWith(`${output}/`)) throw new Error("Input must be a file outside mutable TASK/run output");
  }
  const codeRoot = task.stage === "FORMAL" ? inside(root, request.formal?.codeRoot) : root;
  if (task.stage !== "EXPLORE") {
    if (!request.replay?.codeRevision || !request.replay?.dependencies || !request.inputs.length) throw new Error("VALIDATE/FORMAL require replay code/dependency references and inputs");
    if (git(codeRoot, ["rev-parse", "HEAD"]) !== request.replay.codeRevision) throw new Error("Code revision differs from the approved request");
    if (task.stage === "VALIDATE") {
      contractTable(task, "Validation Contract", ["Hypothesis / Claim", "Treatment", "Baseline", "Endpoint", "Population / Inferential Unit", "Seed Schedule", "Code / Config Revision", "Failure / Stop Rule"]);
      if (typeof request.replay.implementationChanged !== "boolean") throw new Error("VALIDATE must declare whether controlled implementation/config changed");
      if (request.replay.implementationChanged && git(root, ["branch", "--show-current"]) !== `task/${task.taskId}`) throw new Error("VALIDATE implementation changes must run on the task branch");
      // TASK Progress is intentionally mutable; code/config must already be a replayable revision.
      if (git(root, ["status", "--porcelain", "--untracked-files=all", "--", ".", ":(exclude)state/TASK.md"])) throw new Error("VALIDATE needs clean code/config; commit the approved revision first");
    }
  }
  if (task.stage === "FORMAL") {
    contractTable(task, "Experiment Contract", ["Claim", "Frozen Protocol", "Holdout / Data", "Metric / Decision Rule", "Immutable Request / Environment Binding", "Evidence Location"]);
    const binding = request.formal;
    if (!binding || binding.codeCommit !== request.replay.codeRevision || !task.sections.Context.includes(`Frozen commit: \`${binding.codeCommit}\``)) throw new Error("FORMAL frozen commit binding is missing or inconsistent");
    const protocolId = binding.protocol?.match(/^docs\/protocols\/(P-\d+)[^/]*\.md$/)?.[1];
    if (!protocolId || !task.sections.Context.includes(`\`${binding.protocol}\``) || !task.sections["Experiment Contract"].includes(protocolId) ||
        !task.sections["Experiment Contract"].includes(task.requestPath)) throw new Error("FORMAL Protocol/request pointers must match TASK and Experiment Contract");
    if (resolve(git(codeRoot, ["rev-parse", "--show-toplevel"])) !== resolve(codeRoot)) throw new Error("FORMAL codeRoot must be a separate frozen checkout");
    if (git(codeRoot, ["status", "--porcelain", "--untracked-files=all"])) throw new Error("FORMAL requires a clean frozen checkout");
    const validation = readJson(inside(root, binding.validationRun));
    if (validation.request?.stage !== "VALIDATE" || validation.request.taskId !== task.taskId || validation.status !== "completed" ||
        !validation.results?.length || validation.cursor !== validation.request.steps.length || validation.results.some(result => !result.passed)) throw new Error("FORMAL needs a completed controlled VALIDATE run for this task");
    if (!request.inputs.includes(binding.validationRun)) throw new Error("FORMAL must bind its prior VALIDATE record as an input");
    for (const key of ["protocol", "holdout", "dependencyLock", "dataManifest", "environmentContract"]) {
      if (!request.inputs.includes(binding[key])) throw new Error(`FORMAL needs ${key} in inputs`);
    }
    if (!/^\s*(?:- )?Status:\s*`?Frozen`?\s*$/m.test(readFileSync(inside(root, binding.protocol), "utf8"))) throw new Error("Protocol is not Frozen");
    if (binding.hostFingerprint !== hostFingerprint()) throw new Error("FORMAL host fingerprint mismatch");
    for (const input of request.inputs) {
      if (!/^[a-f0-9]{64}$/.test(binding.sha256?.[input] ?? "") || sha(readFileSync(inside(root, input))) !== binding.sha256[input]) throw new Error(`FORMAL input hash mismatch: ${input}`);
    }
  }
  return output;
}

function snapshotInputs(root, request, output) {
  return request.inputs.map((path, index) => {
    const data = readFileSync(inside(root, path)), snapshot = `inputs/${index}`;
    mkdirSync(resolve(output, "inputs"), { recursive: true });
    writeFileSync(resolve(output, snapshot), data, { flag: "wx" });
    return { path, snapshot, ...(request.stage === "FORMAL" ? { sha256: sha(data) } : {}) };
  });
}

function persist(root, output, state) {
  state.updatedAt = new Date().toISOString();
  writeJson(resolve(output, "workflow-state.json"), state);
  const task = requireActive(root);
  if (same(task.guard, state.taskGuard)) {
    updateProgress(root, state.taskGuard, `${state.request.outputDir}/workflow-state.json`, `${state.status}; ${state.nextAction}`);
  }
}

function stopped(root, output, state, status, reason) {
  state.status = status;
  state.reason = reason;
  state.nextAction = "Stop and review the recorded failure; no retry or later step is authorized by this run.";
  state.pendingMemory = `${state.request.outputDir}/failure-memory.pending.json`;
  // This is an execution observation, never an inferred Rejected/Refuted decision.
  const candidate = { version: 1, kind: "pending-authority-review", evidenceEligible: false,
    taskId: state.request.taskId, stage: state.request.stage, question: state.taskGuard.Question,
    authorityContext: state.taskGuard.Context, run: state.request.outputDir,
    reason, observation: state.results.at(-1) ?? null,
    nextAction: "Review the authoritative Idea/Experiment first, then prepare and explicitly accept a Wiki index proposal." };
  const candidatePath = resolve(output, "failure-memory.pending.json");
  if (!existsSync(candidatePath)) writeJson(candidatePath, candidate, true);
  persist(root, output, state);
  return state;
}

function acquire(output) {
  const path = resolve(output, "workflow.lock");
  const fd = openSync(path, "wx");
  writeFileSync(fd, JSON.stringify({ pid: process.pid, host: hostname(), startedAt: new Date().toISOString() }));
  return () => { closeSync(fd); rmSync(path); };
}

export function startFlow(root) {
  const task = requireActive(root);
  if (!task.requestPath) throw new Error("TASK Context must point to the approved Execution request");
  if (task.checkpoint && existsSync(inside(root, task.checkpoint))) {
    const previous = readJson(inside(root, task.checkpoint));
    if (["ready", "running"].includes(previous.status)) throw new Error("An unfinished run already owns the mainline; resume or stop it first");
  }
  const request = readJson(inside(root, task.requestPath));
  git(root, ["rev-parse", "HEAD"]);
  const output = validateRequest(root, request, task);
  mkdirSync(dirname(output), { recursive: true });
  mkdirSync(output); // Never overwrite/reuse a Run ID, including a failed preflight snapshot.
  const releaseLock = acquire(output);
  try {
    const state = { version: 1, requestPath: task.requestPath, request, taskGuard: task.guard,
      status: "ready", reason: null, cursor: 0, reservedCostUnits: 0, elapsedMs: 0,
      results: [], parked: [], inputSnapshots: snapshotInputs(root, request, output),
      nextAction: `Run approved step ${request.steps[0].id}`, evidenceEligible: false, createdAt: new Date().toISOString() };
    writeJson(resolve(output, "request.snapshot.json"), request, true);
    persist(root, output, state);
    return state;
  } finally { releaseLock(); }
}

export function locateFlow(root) {
  const task = requireActive(root);
  if (!task.checkpoint) throw new Error("No workflow checkpoint in TASK Progress; start the approved request first");
  const file = inside(root, task.checkpoint), state = readJson(file);
  if (task.checkpoint !== `${state.request.outputDir}/workflow-state.json` || state.request.taskId !== task.taskId) throw new Error("Checkpoint identity mismatch");
  return { task, state, output: dirname(file) };
}

export function checkFlow(root, located = locateFlow(root), { full = true } = {}) {
  const { task, state, output } = located;
  if (!same(task.guard, state.taskGuard)) throw new Error("Mainline changed: TASK question, scope, budget or contract differs; review before a new run");
  if (task.requestPath !== state.requestPath || !same(readJson(inside(root, task.requestPath)), state.request) ||
      !same(readJson(resolve(output, "request.snapshot.json")), state.request)) throw new Error("Approved execution request changed; this run cannot continue");
  if (!full) return located; // Context restoration reads the control plane, not every raw input.
  validateRequest(root, state.request, task);
  for (const input of state.inputSnapshots) {
    if (!readFileSync(inside(root, input.path)).equals(readFileSync(inside(output, input.snapshot)))) throw new Error(`Approved input changed: ${input.path}; start a reviewed new run`);
  }
  return located;
}

function measurementPass(data, rule) {
  let value = data;
  for (const key of rule.field.split(".")) {
    if (!value || !Object.hasOwn(value, key)) return false;
    value = value[key];
  }
  if (typeof value !== typeof rule.value || (typeof value === "number" && !Number.isFinite(value))) return false;
  return ({ eq: () => value === rule.value, lt: () => value < rule.value, lte: () => value <= rule.value,
    gt: () => value > rule.value, gte: () => value >= rule.value })[rule.op]();
}

async function runCommand(step, output, timeoutMs, signal, onSpawn) {
  const argv = step.argv.map(arg => arg.replaceAll("{output_dir}", output));
  const stdout = openSync(resolve(output, "stdout.log"), "wx"), stderr = openSync(resolve(output, "stderr.log"), "wx");
  const started = performance.now();
  let child, timedOut = false, aborted = false;
  try {
    return await new Promise((resolveResult) => {
      let error = null, killTimer;
      const kill = () => {
        if (!child?.pid) return;
        try { process.kill(process.platform === "win32" ? child.pid : -child.pid, "SIGTERM"); } catch {}
        killTimer = setTimeout(() => { try { process.kill(process.platform === "win32" ? child.pid : -child.pid, "SIGKILL"); } catch {} }, 200);
      };
      child = spawn(argv[0], argv.slice(1), { cwd: step.cwd, shell: false, detached: process.platform !== "win32",
        env: { ...process.env, HARNESS_OUTPUT_DIR: output }, stdio: ["ignore", stdout, stderr] });
      child.on("spawn", () => {
        try { onSpawn(child.pid); }
        catch (caught) { error = `Cannot persist process identity: ${caught.message}`; kill(); }
      });
      const timer = setTimeout(() => { timedOut = true; kill(); }, timeoutMs);
      const abort = () => { aborted = true; kill(); };
      signal?.addEventListener("abort", abort, { once: true });
      if (signal?.aborted) abort();
      child.on("error", caught => { error = caught.message; });
      child.on("close", (code, closeSignal) => {
        clearTimeout(timer);
        // Always reap the entire process group, including children whose parent exited first.
        if (child.pid && process.platform !== "win32") { try { process.kill(-child.pid, "SIGKILL"); } catch {} }
        clearTimeout(killTimer);
        signal?.removeEventListener("abort", abort);
        resolveResult({ code, signal: closeSignal, timedOut, aborted, error, elapsedMs: Math.ceil(performance.now() - started) });
      });
    });
  } finally { closeSync(stdout); closeSync(stderr); }
}

export async function stepFlow(root, { stepId, signal } = {}) {
  const located = locateFlow(root), { output } = located;
  const releaseLock = acquire(output);
  try {
    const { state } = locateFlow(root);
    if (state.status !== "ready") throw new Error(`Run is ${state.status}; no automatic retry or advancement`);
    const step = state.request.steps[state.cursor];
    if (stepId && stepId !== step.id) throw new Error(`Mainline only permits next step ${step.id}; park other work`);
    try { checkFlow(root); }
    catch (error) { return stopped(root, output, state, "blocked", error.message); }
    const remainingMs = state.request.budget.maxWallTimeMs - state.elapsedMs;
    if (state.reservedCostUnits + step.costUnits > state.request.budget.maxCostUnits || remainingMs < step.timeoutMs) {
      return stopped(root, output, state, "budget-stopped", `Insufficient budget for ${step.id}; expensive work was not launched`);
    }
    if (signal?.aborted) return stopped(root, output, state, "interrupted", "Cancelled before launch");
    const stepOutput = resolve(output, step.id);
    mkdirSync(stepOutput);
    state.status = "running";
    state.nextAction = `Wait for ${step.id}; interruption requires inspection, never blind retry`;
    state.reservedCostUnits += step.costUnits;
    persist(root, output, state); // Reserve before launching, survive crashes without re-spending.
    const execution = await runCommand({ ...step, cwd: state.request.stage === "FORMAL" ? inside(root, state.request.formal.codeRoot) : resolve(root) }, stepOutput, Math.min(step.timeoutMs, remainingMs), signal,
      pid => { state.runningProcess = { pid, host: hostname() }; persist(root, output, state); });
    delete state.runningProcess;
    state.elapsedMs += execution.elapsedMs;
    let measurements = null, failure = execution.error;
    if (execution.code !== 0 || execution.timedOut || execution.aborted) failure = failure ?? "Command failed, timed out or was cancelled";
    if (!failure) {
      try {
        const result = inside(stepOutput, step.resultFile);
        if (statSync(result).size > 1024 * 1024) throw new Error("Result JSON exceeds 1 MiB; keep raw data in artifacts");
        measurements = readJson(result);
        if (!step.pass.every(rule => measurementPass(measurements, rule))) failure = "Predeclared pass condition failed or a measurement is missing";
      } catch (error) { failure = `Invalid/missing result: ${error.message}`; }
    }
    // Commands cannot mutate their contract/inputs and still unlock the next step.
    try { checkFlow(root); } catch (error) { failure = `Execution binding changed: ${error.message}`; }
    const result = { step: step.id, passed: !failure, execution,
      result: `${state.request.outputDir}/${step.id}/${step.resultFile}`, failure };
    if (state.request.stage === "FORMAL") {
      try {
        result.sha256 = Object.fromEntries(["stdout.log", "stderr.log", step.resultFile]
          .filter(file => existsSync(resolve(stepOutput, file))).map(file => [file, sha(readFileSync(inside(stepOutput, file)))]));
      } catch (error) { failure = `Cannot bind FORMAL output: ${error.message}`; result.failure = failure; result.passed = false; }
    }
    writeJson(resolve(stepOutput, "execution.json"), result, true);
    state.results.push(result);
    if (failure) return stopped(root, output, state, execution.aborted ? "interrupted" : "failed", failure);
    state.cursor += 1;
    state.status = state.cursor === state.request.steps.length ? "completed" : "ready";
    state.nextAction = state.status === "completed" ? "Review results against Done When; stage and scientific acceptance stay unchanged" : `Run approved step ${state.request.steps[state.cursor].id}`;
    persist(root, output, state);
    return state;
  } finally { releaseLock(); }
}

export function parkWork(root, proposal) {
  if (typeof proposal !== "string" || !proposal.trim() || proposal.length > 1000) throw new Error("Park one short proposal (1–1000 characters)");
  const { output } = locateFlow(root), releaseLock = acquire(output);
  try {
    const { state } = checkFlow(root);
    state.parked.push({ proposal, status: "not-authorized", createdAt: new Date().toISOString() });
    persist(root, output, state);
    return { parked: state.parked.length, nextAction: state.nextAction };
  } finally { releaseLock(); }
}

export function recoverInterrupted(root) {
  const { output, state: initial } = locateFlow(root), lock = resolve(output, "workflow.lock");
  if (initial.runningProcess) {
    if (initial.runningProcess.host !== hostname()) throw new Error("Child belongs to another host; inspect it before recovery");
    try {
      process.kill(process.platform === "win32" ? initial.runningProcess.pid : -initial.runningProcess.pid, 0);
      throw new Error("A command process group is still alive; inspect/stop it through the host before recovery");
    } catch (error) { if (error.code !== "ESRCH") throw error; }
  }
  if (existsSync(lock)) {
    const owner = readJson(lock);
    if (owner.host !== hostname()) throw new Error("Lock belongs to another host; inspect that host before recovery");
    try { process.kill(owner.pid, 0); throw new Error("Workflow process is still alive"); }
    catch (error) { if (error.code !== "ESRCH") throw error; }
    rmSync(lock); // Explicit recovery only; never steals a live lock or repeats a command.
  }
  const releaseLock = acquire(output);
  try {
    const { state } = locateFlow(root);
    if (state.status !== "running") return state; // Crash after settlement: only the dead owner's lock needed cleanup.
    return stopped(root, output, state, "interrupted", "Executor disappeared; reserved cost retained, outputs require review, command not retried");
  } finally { releaseLock(); }
}

export function stopFlow(root, reason) {
  if (!reason?.trim() || reason.length > 1000) throw new Error("Supply a short reason for stopping this run");
  const { output } = locateFlow(root), releaseLock = acquire(output);
  try {
    const { state } = locateFlow(root);
    if (state.status !== "ready") throw new Error("Only a ready run can be stopped; interrupt a live process through its host");
    return stopped(root, output, state, "stopped", reason);
  } finally { releaseLock(); }
}
