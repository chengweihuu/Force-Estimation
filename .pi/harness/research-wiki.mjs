import { closeSync, existsSync, openSync, readFileSync, readdirSync, renameSync, rmSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { randomUUID } from "node:crypto";
import { inside, readJson, writeJson } from "./research-context.mjs";

const WIKI = "docs/RESEARCH_WIKI.md";
const COLUMNS = ["Memory ID", "Status", "Authoritative object", "Evidence pointer", "Applicable boundary", "Reusable lesson", "Restart condition"];
const KEYS = ["id", "status", "authority", "evidence", "boundary", "lesson", "restart"];
const cell = (value) => value.replaceAll("&", "&amp;").replaceAll("|", "&#124;").replaceAll("\n", " ");
const uncell = (value) => value.replaceAll("&#124;", "|").replaceAll("&amp;", "&");

export function wikiRows(root) {
  const text = readFileSync(inside(root, WIKI), "utf8");
  const lines = text.split("\n");
  const header = lines.findIndex(line => line.split("|").slice(1, -1).map(value => value.trim()).join("\0") === COLUMNS.join("\0"));
  if (header < 0 || !/^\|(?:[ :\-]+\|){7}$/.test(lines[header + 1] ?? "")) throw new Error("Malformed Research Wiki table");
  const rows = [], ids = new Set();
  for (let i = header + 2; i < lines.length && lines[i].startsWith("|"); i++) {
    if (lines[i].includes("_Add after an authority object exists_")) continue;
    const values = lines[i].split("|").slice(1, -1).map(value => uncell(value.trim()));
    if (values.length !== 7 || values.some(value => !value) || ids.has(values[0])) throw new Error(`Malformed/duplicate Wiki row at line ${i + 1}`);
    ids.add(values[0]);
    rows.push(Object.fromEntries(KEYS.map((key, index) => [key, values[index]])));
  }
  return { text, lines, header, rows };
}

function pointer(value) {
  const link = value.match(/^\[[^\]]+\]\(([^)]+)\)$/);
  return (link?.[1] ?? value.replaceAll("`", "")).split("#")[0];
}

function authorityStatus(root, value) {
  const path = pointer(value);
  if (!/^docs\/ideas\/I-\d+[^/]*\.md$/.test(path) && !/^experiments\/E-\d+[^/]*\/README\.md$/.test(path)) throw new Error("Failure authority must be an Idea or Experiment, using a repository-relative path");
  const text = readFileSync(inside(root, path), "utf8");
  const key = path.startsWith("docs/ideas/") ? "Status" : "ProtocolOutcome";
  const values = [...text.matchAll(new RegExp(`^(?:- )?${key}:\\s*\x60?([A-Za-z]+)\x60?\\s*$`, "gm"))];
  if (values.length !== 1) throw new Error(`Authority needs exactly one ${key}: ${path}`);
  const status = values[0][1];
  if (path.startsWith("docs/ideas/") ? status !== "Rejected" : !["Refuted", "Inconclusive"].includes(status)) throw new Error(`Authority has no indexable failure status: ${status}`);
  return { path, text, status };
}

function validateEntry(root, entry) {
  for (const key of KEYS) if (typeof entry[key] !== "string" || !entry[key].trim() || entry[key].length > 600 || /[\r\n]/.test(entry[key])) throw new Error(`Wiki ${key} must be one short nonempty line`);
  if (!/^M-\d+$/.test(entry.id)) throw new Error("Memory ID must be M-<digits>");
  const authority = authorityStatus(root, entry.authority);
  if (entry.status !== authority.status) throw new Error("Wiki status must match the authoritative object");
  if (!["None", "无"].includes(entry.evidence)) {
    const evidencePath = pointer(entry.evidence);
    // A transcript or a raw failed probe is not accepted Evidence.
    if (!/^experiments\/E-\d+[^/]*\/README\.md$/.test(evidencePath) || !existsSync(inside(root, evidencePath))) throw new Error("Evidence must point to an authoritative Experiment, or None");
    const evidence = readFileSync(inside(root, evidencePath), "utf8");
    if (!/^(?:- )?ResearcherVerification:\s*Accepted\s*$/m.test(evidence) || !/^(?:- )?ExperimentStatus:\s*Completed\s*$/m.test(evidence) || !/^(?:- )?Protocol:\s*P-\d+/m.test(evidence)) throw new Error("Evidence pointer needs a completed, researcher-accepted protocol Experiment; otherwise use None");
  }
  return authority;
}

export function queryWiki(root, query = "", limit = 5) {
  if (!Number.isInteger(limit) || limit < 1 || limit > 20) throw new Error("Memory query limit must be 1–20");
  const { rows } = wikiRows(root), q = query.toLocaleLowerCase();
  const terms = [...new Set(q.match(/[a-z0-9_-]+|[\p{Script=Han}]+/gu) ?? [])];
  // Lexical retrieval only. Chinese bigrams allow a longer question to find a short boundary phrase.
  const tokens = terms.flatMap(term => /\p{Script=Han}/u.test(term) && term.length > 2
    ? Array.from({ length: term.length - 1 }, (_, index) => term.slice(index, index + 2)) : [term]);
  const scored = rows.map(row => ({ row, score: tokens.reduce((n, token) => n + Number(Object.values(row).join(" ").toLocaleLowerCase().includes(token)), 0) }))
    .filter(item => !q.trim() || item.score > 0).sort((a, b) => b.score - a.score || a.row.id.localeCompare(b.row.id));
  return { query, total: scored.length, omitted: Math.max(0, scored.length - limit), rows: scored.slice(0, limit).map(item => item.row), evidenceEligible: false };
}

export function auditWiki(root) {
  const { rows } = wikiRows(root), errors = [], missing = [], authorities = [];
  for (const row of rows) {
    try { validateEntry(root, { ...row, status: row.status.replaceAll("`", "") }); }
    catch (error) { errors.push({ id: row.id, reason: error.message }); }
  }
  const ideas = inside(root, "docs/ideas");
  if (existsSync(ideas)) for (const file of readdirSync(ideas).filter(name => /^I-\d+.*\.md$/.test(name))) authorities.push(`docs/ideas/${file}`);
  const experiments = inside(root, "experiments");
  if (existsSync(experiments)) for (const file of readdirSync(experiments).filter(name => /^E-\d+/.test(name))) {
    if (existsSync(inside(root, `experiments/${file}/README.md`))) authorities.push(`experiments/${file}/README.md`);
  }
  for (const path of authorities) {
    try {
      const authority = authorityStatus(root, path);
      if (!rows.some(row => pointer(row.authority) === path && row.status.replaceAll("`", "") === authority.status)) missing.push({ authority: path, status: authority.status });
    } catch (error) {
      if (!error.message.startsWith("Authority has no indexable failure status")) errors.push({ authority: path, reason: error.message });
    }
  }
  return { ok: !errors.length && !missing.length, missing, errors };
}

function proposalPath(root, name) {
  if (!/^(runs|scratch)\/H-\d+[^/]*\/.+\.json$/.test(name)) throw new Error("Memory drafts belong in task scratch/run output");
  return inside(root, name);
}

export function prepareWiki(root, entry, destination) {
  const authority = validateEntry(root, entry), wiki = wikiRows(root);
  const duplicate = wiki.rows.find(row => row.id === entry.id || pointer(row.authority) === authority.path);
  if (duplicate && duplicate.id !== entry.id) throw new Error(`Authority already indexed as ${duplicate.id}; update that row`);
  if (duplicate && pointer(duplicate.authority) !== authority.path) throw new Error("Cannot reuse a Memory ID for a different authority");
  // Markdown links resolve from docs/RESEARCH_WIKI.md. Store repository paths as inline code.
  const normalized = { ...entry, authority: `\`${authority.path}\``, evidence: ["None", "无"].includes(entry.evidence) ? entry.evidence : `\`${pointer(entry.evidence)}\`` };
  const row = `| ${KEYS.map(key => cell(normalized[key])).join(" | ")} |`;
  const lines = [...wiki.lines];
  const index = lines.findIndex(line => line.startsWith(`| ${entry.id} |`));
  let before = null;
  if (index >= 0) { before = lines[index]; lines[index] = row; }
  else {
    let end = wiki.header + 2;
    while (end < lines.length && lines[end].startsWith("|")) end++;
    lines.splice(end, 0, row);
  }
  const proposal = { version: 1, kind: "wiki-index-proposal", entry: normalized,
    authorityText: authority.text, beforeWiki: wiki.text, afterWiki: lines.join("\n"),
    diff: `${before ? `- ${before}\n` : ""}+ ${row}`, evidenceEligible: false };
  writeJson(proposalPath(root, destination), proposal, true);
  return { proposal: destination, diff: proposal.diff, requiresUserAcceptance: true };
}

export function applyWiki(root, name, { accepted = false } = {}) {
  if (!accepted) throw new Error("Show the exact Wiki proposal and obtain user acceptance before applying");
  const proposal = readJson(proposalPath(root, name));
  if (proposal.version !== 1 || proposal.kind !== "wiki-index-proposal") throw new Error("Not a Wiki index proposal");
  const authority = validateEntry(root, proposal.entry), path = inside(root, WIKI);
  const lock = `${path}.lock`, fd = openSync(lock, "wx");
  try {
    if (authority.text !== proposal.authorityText || readFileSync(path, "utf8") !== proposal.beforeWiki) throw new Error("Wiki or authority changed since review; prepare a fresh proposal");
    const existing = wikiRows(root).rows;
    if (existing.some(row => (row.id === proposal.entry.id) !== (pointer(row.authority) === authority.path))) throw new Error("Cannot replace another authority or duplicate its Memory ID");
    // Reconstruct instead of trusting afterWiki supplied by an executor.
    const row = `| ${KEYS.map(key => cell(proposal.entry[key])).join(" | ")} |`;
    const lines = proposal.beforeWiki.split("\n"), oldIndex = lines.findIndex(line => line.startsWith(`| ${proposal.entry.id} |`));
    if (oldIndex >= 0) lines[oldIndex] = row;
    else {
      const { header } = wikiRows(root);
      let end = header + 2;
      while (end < lines.length && lines[end].startsWith("|")) end++;
      lines.splice(end, 0, row);
    }
    if (lines.join("\n") !== proposal.afterWiki) throw new Error("Proposal includes changes beyond its single index row");
    const temp = `${path}.${randomUUID()}.tmp`;
    writeFileSync(temp, proposal.afterWiki, { flag: "wx" });
    renameSync(temp, path);
    return { applied: proposal.entry.id, authority: authority.path, evidenceEligible: false };
  } finally { closeSync(fd); rmSync(lock); }
}
