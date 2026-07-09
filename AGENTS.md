# AGENTS.md

Instructions for coding agents working in this repository.

## Agent skills

### Issue tracker

Issues live as markdown files under `.scratch/<feature-slug>/` in this repo.
There is no external tracker, and pull requests are not a triage surface.
See `docs/agents/issue-tracker.md`.

### Triage labels

The five canonical triage roles are used verbatim — `needs-triage`,
`needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix` — recorded as a
`Status:` line in each issue file. See `docs/agents/triage-labels.md`.

### Domain docs

Multi-context: `CONTEXT-MAP.md` at the repo root points at one `CONTEXT.md`
per ROS 2 package. See `docs/agents/domain.md`.
