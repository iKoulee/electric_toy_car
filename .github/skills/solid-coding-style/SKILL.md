---
name: solid-coding-style
description: "Apply SOLID principles during design, implementation, and refactoring. Use for new features, architecture reviews, coupling reduction, and testability improvements. Behavior is preserved by default, but beneficial behavior improvements are allowed when justified. Keywords: SOLID, SRP, OCP, LSP, ISP, DIP, refactor, design review."
argument-hint: "Task or file to apply SOLID to (for example: refactor vehicle/src/main.rs motor control)"
---

# SOLID Coding Style

Use this skill to produce code that is easier to reason about, safer to change, and simpler to test.

## When To Use
- Implementing a new feature where responsibilities are unclear.
- Refactoring large modules with mixed concerns.
- Reviewing pull requests for architecture quality.
- Designing interfaces between hardware, protocol, and control logic.

## Inputs
- Target scope: file, module, or feature.
- Constraints: performance, memory, and platform limits.
- Safety requirements and non-functional requirements.

## Input Validation and Guardrails
- If no concrete code, file, or module is provided, stop and ask for specific input before proposing edits.
- If the target language is unclear, ask for language confirmation before generating code changes.
- Do not infer framework, language, or architecture details without evidence from provided context.
- If context is partial, provide a scoped assessment only for visible code and clearly state unknowns.

## Boundaries
- This skill focuses on SOLID-driven implementation and refactoring workflow.
- This skill does not enforce mandatory TDD for every task.
- This skill does not replace full architecture governance or product decision processes.

## Pre-Change Checklist
- Define scope and acceptance criteria for the requested change.
- Capture constraints and non-functional requirements that must hold.
- Identify behavior-preserving default and what qualifies as a justified behavior improvement.

## Procedure
1. Map responsibilities
- List the current responsibilities in the target code.
- Mark each as domain logic, IO/hardware, orchestration, validation, or formatting.

2. Apply SRP (Single Responsibility Principle)
- Split code where one unit has more than one reason to change.
- Keep orchestration separate from hardware and protocol details.

3. Apply OCP (Open/Closed Principle)
- Prefer extension points (traits/interfaces, strategy structs, callbacks) instead of modifying stable core logic.
- Add new behavior via composition, not branching across unrelated concerns.

4. Apply LSP (Liskov Substitution Principle)
- Ensure implementations preserve interface contracts.
- Avoid surprising side effects or stricter preconditions in implementations.

5. Apply ISP (Interface Segregation Principle)
- Break large interfaces into role-specific interfaces.
- Keep consumers dependent on the smallest surface they need.

6. Apply DIP (Dependency Inversion Principle)
- Depend on abstractions for boundaries (drivers, transports, repositories).
- Inject concrete dependencies at composition roots.

7. Re-check system constraints
- Respect the project's runtime, performance, and reliability constraints.
- Keep critical paths predictable and avoid unnecessary overhead.
- Ensure failures at boundaries are handled explicitly.

8. Verify behavior and quality
- Preserve external behavior by default; if improving behavior, state the rationale and expected user impact.
- Add or update tests around boundary interfaces and key invariants.
- Validate naming, cohesion, coupling, and error paths.

## In-Change Checklist
- SRP: Each changed unit has one clear reason to change.
- OCP: New behavior is added via extension seams where practical.
- LSP: Implementations continue to satisfy contract expectations.
- ISP: Consumers depend only on methods they actually use.
- DIP: Core policy depends on abstractions, not concrete details.

## Decision Rules
- If splitting increases complexity without improving change isolation, keep the simpler structure and document why.
- If abstraction adds no second use case, start with a small seam and defer generalization.
- If SOLID and product goals conflict, prioritize correctness, user impact, and reliability.
- If performance constraints conflict with purity, choose predictable performance and keep abstractions lightweight.

## Completion Criteria
- Each unit has one clear reason to change.
- New behavior can be added with minimal edits to stable code.
- Interface contracts are explicit and consistently honored.
- Consumers depend only on methods they use.
- Core logic depends on abstractions, not concrete peripherals.
- Error handling and fail-safe paths are explicit.
- Tests or validation steps cover the changed boundaries.
- At least one evidence item is provided (test output, static check result, or coupling/cohesion before-after rationale).

## Post-Change Checklist
- Verify tests and checks relevant to changed boundaries.
- Call out any residual SOLID debt and why it was deferred.
- Summarize risks, especially around contracts and extension points.

## Red Flags
- A single module keeps accumulating unrelated responsibilities.
- New feature requires editing many unrelated files (shotgun edits).
- Core logic depends directly on concrete adapters or frameworks.
- Interfaces grow to include methods many consumers do not need.
- Subtype or implementation changes silently break caller expectations.

## Output Format
When this skill is invoked and material issues are found, produce:
1. A short SOLID assessment of current code.
2. Proposed refactor plan grouped by SRP/OCP/LSP/ISP/DIP.
3. Concrete code edits.
4. Validation summary (tests run, remaining risks, follow-ups).

If the code already satisfies SOLID principles (do NOT fabricate violations):
1. A compliance summary stating which principles are satisfied with specific evidence.
2. Any minor improvement opportunities flagged as strictly optional.
3. A clear confirmation that no code edits are required.

If required inputs are missing, produce instead:
1. A brief blocked status stating what is missing.
2. Exact files/modules or code snippets needed.
3. Up to three focused clarification questions.
