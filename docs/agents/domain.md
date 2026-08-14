# Domain docs

This repository uses a single-context domain-document layout.

## Before exploring

- Read `CONTEXT.md` at the repository root when it exists.
- Read relevant decisions under `docs/adr/` when they exist.
- If either location is absent, proceed silently; producer skills create domain
  documents lazily when terms or decisions are resolved.

## Vocabulary

Use the terms defined in `CONTEXT.md` when naming domain concepts. Avoid drifting
to synonyms that the glossary rejects. A missing term may indicate either an
invented concept or a genuine documentation gap.

## Architectural decisions

Surface any contradiction with an existing ADR explicitly instead of silently
overriding it.
