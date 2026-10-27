import { lstatSync, mkdirSync, readFileSync, renameSync, writeFileSync } from "node:fs";
import { dirname, isAbsolute, relative, resolve, sep } from "node:path";
import { randomUUID } from "node:crypto";

export function inside(root, name) {
  if (typeof name !== "string" || !name || isAbsolute(name)) throw new Error("Expected a repository-relative path");
  const target = resolve(root, name), rel = relative(resolve(root), target);
  if (!rel || rel === ".." || rel.startsWith(`..${sep}`)) throw new Error(`Path escapes repository: ${name}`);
  let part = resolve(root);
  for (const segment of rel.split(sep)) {
    part = resolve(part, segment);
    try { if (lstatSync(part).isSymbolicLink()) throw new Error(`Symlink is not a workflow path: ${name}`); }
    catch (error) { if (error.code !== "ENOENT") throw error; }
  }
  return target;
}

export const readJson = (path) => JSON.parse(readFileSync(path, "utf8"));
function writeText(path, text, exclusive = false) {
  mkdirSync(dirname(path), { recursive: true });
  if (exclusive) return writeFileSync(path, text, { flag: "wx" });
  const temp = `${path}.${randomUUID()}.tmp`;
  writeFileSync(temp, text, { flag: "wx" });
  renameSync(temp, path);
}
export function writeJson(path, value, exclusive = false) {
  writeText(path, `${JSON.stringify(value, null, 2)}\n`, exclusive);
}

export function sections(text) {
  const matches = [...text.matchAll(/^## (.+)$/gm)], result = {};
  matches.forEach((match, index) => {
    const name = match[1].trim();
    if (Object.hasOwn(result, name)) throw new Error(`Duplicate TASK section: ${name}`);
    result[name] = text.slice(match.index + match[0].length, matches[index + 1]?.index).trim();
  });
  return result;
}

function field(text, key, optional = false) {
  const values = text.split("\n").filter(line => line.startsWith(`- ${key}:`))
    .map(line => line.slice(key.length + 3).trim());
  if (optional && !values.length) return null;
  if (values.length !== 1 || !values[0]) throw new Error(`TASK needs exactly one ${key}`);
  return values[0];
}

export function readTask(root) {
  const raw = readFileSync(inside(root, "state/TASK.md"), "utf8");
  const inactive = "# Current Task\n\nStatus: Inactive\n\n下一步：根据 CURRENT 进入 THINK；用户确认计划后再替换本文件。";
  if (raw.trim() === inactive) return { active: false, raw };
  const statuses = [...raw.matchAll(/^Status: (\S+)\s*$/gm)];
  if (statuses.length !== 1 || statuses[0][1] !== "Active") throw new Error("TASK is missing or malformed");
  const s = sections(raw);
  for (const key of ["Identity", "Question", "Context", "Allowed Changes", "Non-goals", "Budget", "Done When", "Stop and Ask", "Outputs", "Information Gain"]) {
    if (!s[key]) throw new Error(`TASK needs ${key}`);
  }
  const requiredOrder = ["Identity", "Question", "Context", "Allowed Changes", "Non-goals", "Budget", "Done When", "Stop and Ask", "Outputs", "Information Gain"];
  if (Object.keys(s).filter(key => requiredOrder.includes(key)).join("|") !== requiredOrder.join("|")) throw new Error("TASK sections are out of order");
  const taskId = field(s.Identity, "Task ID"), stage = field(s.Identity, "Stage");
  if (!/^H-\d+[\w-]*$/.test(taskId) || !["EXPLORE", "VALIDATE", "FORMAL"].includes(stage)) throw new Error("Invalid TASK identity/stage");
  const progress = field(s.Identity, "Progress");
  field(s.Identity, "Research objects");
  for (const key of ["Files", "Time / Compute", "Robot runs"]) field(s.Budget, key);
  for (const key of ["Scientific", "Engineering"]) field(s["Information Gain"], key);
  const requestValue = field(s.Context, "Execution request", true);
  if (requestValue && !/^`[^`]+`$/.test(requestValue)) throw new Error("Execution request must be one backtick-quoted path");
  const checkpoint = progress.match(/workflow: `([^`]+\/workflow-state\.json)`/);
  // Mutable progress and closeout findings are not execution authority.
  const guard = Object.fromEntries(Object.entries(s).filter(([key]) => key !== "Information Gain")
    .map(([key, value]) => [key, key === "Identity" ? value.replace(/^- Progress:.*$/m, "").trim() : value]));
  return { active: true, raw, sections: s, taskId, stage, progress, guard,
    requestPath: requestValue?.slice(1, -1), checkpoint: checkpoint?.[1],
    robotRuns: field(s.Budget, "Robot runs") };
}

export function requireActive(root) {
  const task = readTask(root);
  if (!task.active) throw new Error("Scientific execution requires an approved Active TASK; ordinary maintenance needs no TASK");
  return task;
}

export function updateProgress(root, guard, checkpoint, message) {
  const task = requireActive(root);
  if (JSON.stringify(task.guard) !== JSON.stringify(guard)) throw new Error("TASK contract changed; refusing to update Progress");
  const text = task.raw.replace(/^- Progress:.*$/m, `- Progress: workflow: \`${checkpoint}\`; ${message}`);
  writeText(inside(root, "state/TASK.md"), text);
}

export function contextPacket(root) {
  const task = readTask(root);
  if (!task.active) return { active: false, nextAction: "普通维护按用户授权执行；科学研究先确定一个问题。" };
  const packet = { active: true, taskId: task.taskId, stage: task.stage, question: task.sections.Question,
    researchObjects: field(task.sections.Identity, "Research objects"), authorityContext: task.sections.Context,
    nonGoals: task.sections["Non-goals"], allowedChanges: task.sections["Allowed Changes"],
    budget: task.sections.Budget, doneWhen: task.sections["Done When"], stopAndAsk: task.sections["Stop and Ask"],
    progress: task.progress, request: task.requestPath ?? null };
  if (task.checkpoint) {
    const state = readJson(inside(root, task.checkpoint));
    if (state.request.taskId !== task.taskId || task.checkpoint !== `${state.request.outputDir}/workflow-state.json`) throw new Error("Checkpoint identity mismatch");
    packet.checkpoint = task.checkpoint;
    packet.contractChanged = JSON.stringify(state.taskGuard) !== JSON.stringify(task.guard);
    packet.status = state.status;
    packet.reason = state.reason;
    packet.lastResult = state.results.at(-1) ?? null;
    packet.nextAction = packet.contractChanged ? "Stop: TASK changed; review before starting a new run." : state.nextAction;
    packet.parked = { count: state.parked.length, pointer: task.checkpoint };
    packet.pendingMemory = state.pendingMemory ?? null;
  }
  return packet;
}

export function boundedContext(packet, maxCharacters = 12000) {
  const serialized = JSON.stringify(packet);
  if (serialized.length <= maxCharacters) return serialized;
  // No silent truncation of a scientific constraint: require targeted retrieval instead.
  return JSON.stringify({ active: packet.active, taskId: packet.taskId, stage: packet.stage,
    contextIncomplete: true, characters: serialized.length, task: "state/TASK.md", checkpoint: packet.checkpoint,
    nextAction: "Stop and read the exact TASK sections/checkpoint needed for this action; the full context packet exceeds the injection limit." });
}
