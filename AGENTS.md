# Humanoid SDK — agent instructions

Help the user install, deploy and use this SDK with as little manual work as
possible. Respond in the user's language. This file is the single instruction
source; `CLAUDE.md` must remain a relative symlink to `AGENTS.md`.

## Start here

- Read `README.md` for the supported release, installation commands and customer
  documentation. Follow the linked SDK documentation for message contracts,
  deployment and EXTERNAL configuration; do not invent commands or topic fields.
- Customer documentation is the published Feishu knowledge base linked in the
  README. Until the SDK pages are published, state that they are unavailable;
  do not invent a page URL or substitute the documentation source repository.
- If the README marks the release as in preparation, or its required package,
  example or manifest is absent, stop installation and explain what is missing.
  The workflow below describes the delivery contract, not evidence that this
  checkout already contains an installable SDK.
- Inspect the current checkout and preserve local changes. Do not require the
  robot monorepo, private schema source, Bazel or an agent-specific plugin for a
  customer's quick start. The Python import name is `locomotion_aorta`.
- Ask only for missing information that changes the next step: local vs robot
  installation, exact target host, target directory and compatible robot release.
  Never reuse a robot hostname, slot or safety confirmation from an old session.
- A request for quick start means installation and read-only verification first,
  not permission to move a robot, replace its OTA image or restart services.

## Installation and deployment workflow

1. Inspect the selected target's OS, architecture, Python version (3.10+) and
   existing installation. Verify the SDK release manifest against the robot's
   deployed Aorta/schema revision; do not choose wheels solely by date or latest
   tag. Use the architecture-matching Linux `aorta_sdk` wheel and matching
   `aorta_msgs` wheel. Obtain release artifacts through the user's authorized
   access; never embed credentials in commands, reports or committed files.
2. Explain the target directory and changes before installing. Use a dedicated
   virtual environment; preserve an existing working installation for rollback.
   On S100, source `/app/script/env.sh` **before** activating the virtual
   environment. Verify that `python` resolves inside that environment.
3. Install the release's declared dependencies, including FlatBuffers. For an
   offline target, use the complete matching wheelhouse and let pip resolve only
   from it. Do not use `--no-deps` to conceal an incomplete bundle. Sourcing
   `env.sh` does not install Python packages.
4. Run `python -m pip check`, import the SDK and real Aorta message bindings,
   inspect installed versions and run the examples' `--help`. Do not call these
   checks proof of a live connection or proof that robot motion is safe.
5. Select the deployment-owned **peer** session profile before constructing any
   Aorta node. On the documented S100 deployment this is
   `/app_param/zenoh/s100_session_peer.json5`, selected with
   `ZENOH_SESSION_CONFIG_URI`. Verify that it exists and actually selects peer;
   a filename alone is not evidence. Do not fall back to client/router mode.
6. With permission to connect to the target, run the documented bounded
   `examples/lowstate_subscriber.py` check. It must not request a control lease or
   publish a motion command. Confirm fresh feedback, expected joint count and
   finite values; report timeout or shutdown warnings rather than hiding them.
7. Stop at the read-only milestone unless the user requests a motion demo.
   Report the next command and its movement before asking for the required
   on-site confirmation. Installation never implies OTA-slot activation.

## Before robot motion

- Require explicit authorization for the specific robot and motion. Require
  current on-site confirmation of reliable support, clear movement and pinch
  zones, an attendant and a usable emergency stop. A CLI confirmation flag is
  not a substitute for that confirmation.
- Explain that the EXTERNAL demo moves joints, requests a 90-second lease, and
  exits through CANCEL to PASSIVE. PASSIVE can let loaded limbs fall; do not
  promise restoration of an action cleared during entry. Inspect the current
  demo's targets and gains rather than assuming another robot's tuning is safe.
- Verify the effective configuration and real joint roster. Use the full
  A-sample configuration. Only intentionally configured fingers may be MOCK in
  the supported configuration; do not hide an absent or faulty actuator by
  changing it to MOCK. Do not raise limits or disable safety checks to make a
  demonstration pass.
- Keep server lease/session ownership, increasing command sequence, watchdog,
  command validation and safe exit intact. ACK acceptance is not completed
  state transition; verify the resulting status.
- Do not automatically retry motion after a fault, communication timeout,
  unexpected movement or incomplete exit. Report the actual state. If safety is
  uncertain, direct the attendant to the emergency stop; software cancellation
  is not a substitute for it.
- Service restart, persistent motor/config changes, OTA installation, slot
  activation and reboot require explicit scope and authorization of their own.

## Cross-repository maintenance (maintainers only)

- The full internal agreement is maintained in `VitaDynamics/vita-robot` at
  `docs/reference/humanoid-sdk-maintenance.zh.md`. This is a maintenance pointer,
  not a customer installation prerequisite; use these local instructions for
  quick start without requiring access to internal repositories.
- For changes to topics, messages, joint mapping, state transitions, lease or
  safety behavior, configuration or installation, name a change owner. In the PR,
  record related server/schema/documentation PRs (or why unaffected), compatible
  versions, verification and release dependencies. Do not duplicate those implementations.
- Keep version/revision, artifact digests and tested robot/configuration evidence
  in the existing release records. A prerelease may be offered for testing before
  device acceptance only with explicit completed checks, gaps and restrictions.
  Recommend normal use only after the agreed acceptance; record accepted deviations
  and exclusions. Never replace published attachments in place; issue a new version.
- New EXTERNAL/SDK test reports must be linked by the change owner from the robot
  repository's `docs/reports/2026-09-11-external-sdk-remaining-verification.zh.md`,
  updating status and date without rewriting original results. Do not duplicate
  internal HIL logs or create a second outstanding-test list here.

## Development and evidence

- The default branch is `main`. Work on feature branches and submit changes
  through pull requests. Synchronize with `git fetch` followed by `git rebase`;
  do not use `git merge` or create merge commits. Do not rewrite, force-push or
  delete the default branch.
- Merge pull requests with **Squash and merge** only, so each PR contributes
  one commit to the default branch. Do not use GitHub's merge-commit or
  rebase-and-merge methods. Obtain explicit user authorization before merging.
- Reuse Aorta transport and generated message wheels; do not copy their runtime
  into this repository or implement a parallel protocol. Keep the SDK thin.
- Maintain reference documentation in the separate documentation source
  repository, but do not expose that repository as a customer entry point or
  prerequisite. Link users only to published Feishu pages. Keep this
  repository's README short; do not duplicate the reference manual or internal
  HIL session logs.
- Run the repository's documented tests after changes. A mocked unit test is
  not a real-binding test; an import test is not a live transport test. Do not
  launch a motion example as part of an automated installation or CI check.
- Finish with a compact report: target, versions/revisions, paths changed,
  checks and results, remaining blocker and next safe step. Mark unavailable
  checks `not_measured`; never claim deployed, connected or motion-tested from
  local build success alone. Avoid secrets and full raw logs in reports.
- Preserve `CLAUDE.md -> AGENTS.md` in Git and release archives. Verify with
  `test -L CLAUDE.md`, `readlink CLAUDE.md` and `cmp AGENTS.md CLAUDE.md`.
  Do not replace the symlink with an independently maintained copy.
