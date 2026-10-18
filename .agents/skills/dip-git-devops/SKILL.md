---
name: dip-git-devops
description: >-
  Git Repository Management, CI/CD Pipeline Automation, and Automated Testing for the DIP Project.
  Covers Python linting, unit tests verifying controllability rank and Riccati closed-loop stability,
  and GitHub Actions workflow configuration (.github/workflows/ci.yml).
---

# Skill: `dip-git-devops`

## Overview
This skill establishes software engineering best practices, continuous integration (CI/CD), automated testing, and repository maintenance for the Double Inverted Pendulum project.

## When to Use
- When initializing or configuring the Git version control repository.
- When performing pre-commit checks or pull request validations.
- When modifying mechanical or physical parameters and confirming that stability criteria remain satisfied:
  1. Kalman controllability matrix rank: $\text{rank}(\mathcal{C}) == 6$.
  2. Algebraic Riccati Equation solution $P$ is symmetric positive-definite ($P = P^T > 0$).
  3. Closed-loop dynamic matrix poles strictly lie in the open left-half complex plane: $\text{Re}(\lambda(\mathbf{A} - \mathbf{B}\mathbf{K})) < 0$.
- When maintaining or extending the GitHub Actions automation workflow in `.github/workflows/ci.yml`.

## Commands & Workflows

### 1. Git Repository Setup & Commit
```bash
git init
git add .
git commit -m "feat: complete double inverted pendulum modeling, simulation, and embedded firmware"
```

### 2. Execute LQR Stability Unit Tests
```bash
pytest tests/test_lqr_stability.py -v
# or using standard unittest:
python tests/test_lqr_stability.py
```

### 3. GitHub Actions CI/CD Pipeline Configuration
Workflow configuration is maintained in: `.github/workflows/ci.yml`
The automated pipeline executes:
- Python syntax checks across all modules.
- Dependency installation from `requirements.txt`.
- Multi-version Python testing (3.10, 3.11, 3.12).
- Unit test execution verifying controllability and asymptotic stability.
- Host compilation verification of embedded C firmware.
