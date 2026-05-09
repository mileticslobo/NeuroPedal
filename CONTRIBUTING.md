# Contributing to cs-motomed-controller

Thank you for considering improvements to cs-motomed-controller. This project repurposes a
legacy Cinesport RECK MOTOmed Viva 1 rehabilitation bike, so every change should
prioritise rider safety and transparency.

## Getting Set Up

1. Fork and clone the repository.
2. Copy credentials template:
   ```bash
   cp arduino_secrets.h.example arduino_secrets.h
   ```
3. Replace the placeholder values in `arduino_secrets.h` with your local Wi-Fi
   SSID and password. Keep this file out of commits — it is ignored by Git.
4. Install the Arduino UNO R4 board support package and required libraries
   (`WiFiS3`).
5. Flash the sketch to a test board before opening a pull request.

## Development Guidelines

- **Safety First** – Document any hardware changes, emergency stop behaviour, or
  control-flow adjustments in the pull request description.
- **Code Style** – Match the existing formatting (two-space indentation, concise
  comments only where needed).
- **Testing** – Verify spasm detection and recovery logic on real hardware or
  provide a convincing simulation trace.
- **Documentation** – Update `README.md` or add docs in `docs/` for new features,
  calibration steps, or hardware adjustments.
- **No Secrets** – Confirm that your branch contains no Wi-Fi credentials, API
  keys, or private health data before submitting.

## Pull Request Checklist

- [ ] Code compiles for `arduino:renesas_uno:unor4wifi`.
- [ ] Manual test plan or log attached to the PR.
- [ ] Documentation updated to describe user-facing changes.
- [ ] Credentials and other sensitive data omitted from commits.

By contributing, you agree that your work will be released under the project’s
license once it is finalised.
