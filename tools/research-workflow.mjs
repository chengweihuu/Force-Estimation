#!/usr/bin/env node
import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { contextPacket, inside } from "../.pi/harness/research-context.mjs";
import { hostFingerprint, locateFlow, parkWork, recoverInterrupted, startFlow, stepFlow, stopFlow } from "../.pi/harness/experiment-flow.mjs";
import { applyWiki, auditWiki, prepareWiki, queryWiki } from "../.pi/harness/research-wiki.mjs";
import { checkReview, exportReview, importReviewFeedback, listReviews } from "../.pi/harness/manual-review.mjs";

export const HELP = `Usage: node tools/research-workflow.mjs [--root PATH] <command>
  context                         Restore question, boundaries, checkpoint and matching failure memories
  flow start                      Snapshot the request referenced by the approved Active TASK
  flow next [step-id]              Run exactly the next declared step
  flow resume                     Continue one step from the durable checkpoint (no retries)
  flow run                        Continue declared steps until completion or the first stop
  flow status                     Read the full run checkpoint
  flow park <proposal>            Preserve a sidetrack without authorizing it
  flow stop <reason>              Stop a ready run before reviewing a new request
  flow recover                    Mark a crashed running checkpoint interrupted; never rerun it
  wiki query [terms]               Retrieve failure-memory index rows
  wiki audit                      Find missing/stale authority pointers and unindexed failures
  wiki propose <entry.json> <draft.json>
                                  Prepare the exact single-row diff in task scratch/run output
  wiki apply <draft.json> --accept Apply only after the user accepts the displayed exact proposal
  review export [brief.json]      Create an on-demand Markdown packet; no model call or upload
  review list                     Find this task's packets and the latest file links
  review check <review-dir>       Check source freshness, missing material and original-file integrity
  review feedback <review-dir> <provider> <model> [raw-feedback-file]
                                  Preserve exact UTF-8 feedback, or register text already pasted into the MD
  host-fingerprint                Print the local FORMAL host binding

Commands execute approved local-compute argv with explicit output_dir. No model, robot,
scientific acceptance, stage promotion or operating-system sandbox is supplied by this CLI.
See docs/research_workflow.md for the request/result schema and a runnable example.`;

export function restoredContext(root, query = "") {
  const context = contextPacket(root);
  return { ...context, memory: queryWiki(root, query || context.question || "") };
}

export async function main(args = process.argv.slice(2)) {
  let root = process.cwd();
  if (args[0] === "--root") { root = resolve(args[1]); args = args.slice(2); }
  const [group, action, ...rest] = args;
  if (!group || ["--help", "help"].includes(group)) { process.stdout.write(`${HELP}\n`); return 0; }
  const controller = new AbortController();
  const cancel = () => controller.abort();
  process.once("SIGINT", cancel);
  process.once("SIGTERM", cancel);
  let result;
  try {
    if (group === "context") result = restoredContext(root);
    else if (group === "host-fingerprint") result = { hostFingerprint: hostFingerprint() };
    else if (group === "flow") {
      if (action === "start") result = startFlow(root);
      else if (["next", "resume"].includes(action)) result = await stepFlow(root, { stepId: rest[0], signal: controller.signal });
      else if (action === "run") {
        do { result = await stepFlow(root, { signal: controller.signal }); } while (result.status === "ready");
      } else if (action === "status") result = locateFlow(root).state;
      else if (action === "park") result = parkWork(root, rest.join(" "));
      else if (action === "stop") result = stopFlow(root, rest.join(" "));
      else if (action === "recover") result = recoverInterrupted(root);
      else throw new Error("Unknown flow action; use --help");
    } else if (group === "wiki") {
      if (action === "query") result = queryWiki(root, rest.join(" "));
      else if (action === "audit") result = auditWiki(root);
      else if (action === "propose") result = prepareWiki(root, JSON.parse(readFileSync(inside(root, rest[0]), "utf8")), rest[1]);
      else if (action === "apply") result = applyWiki(root, rest[0], { accepted: rest[1] === "--accept" && rest.length === 2 });
      else throw new Error("Unknown wiki action; use --help");
    } else if (group === "review") {
      if (action === "export" && rest.length <= 1) result = exportReview(root, rest[0] ? JSON.parse(readFileSync(inside(root, rest[0]), "utf8")) : {});
      else if (action === "list" && !rest.length) result = listReviews(root);
      else if (action === "check" && rest.length === 1) result = checkReview(root, rest[0]);
      else if (action === "feedback" && [3, 4].includes(rest.length)) result = importReviewFeedback(root, rest[0], {
        provider: rest[1], model: rest[2], ...(rest[3] ? { content: readFileSync(resolve(root, rest[3])) } : {}),
      });
      else throw new Error("Unknown review action or incorrect arguments; use --help");
    } else throw new Error("Unknown command; use --help");
    process.stdout.write(`${JSON.stringify(result, null, 2)}\n`);
    return result.ok === false || result.ready === false || ["failed", "blocked", "budget-stopped", "interrupted", "stopped"].includes(result.status) ? 2 : 0;
  } finally {
    process.removeListener("SIGINT", cancel);
    process.removeListener("SIGTERM", cancel);
  }
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  try { process.exitCode = await main(); }
  catch (error) { process.stderr.write(`${error.message}\n`); process.exitCode = 1; }
}
