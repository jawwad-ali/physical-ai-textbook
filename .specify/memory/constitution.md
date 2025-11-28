<!--
SYNC IMPACT REPORT
==================
Version Change: 0.0.0 → 1.0.0 (Initial constitution)
Created: 2025-11-28
Ratification: 2025-11-28

Principles Established:
- I. Educational Clarity
- II. Structured Learning Progression
- III. Code Quality & Reproducibility
- IV. Visual & Multimodal Learning
- V. Interactive Engagement
- VI. Technical Accuracy & Currency
- VII. Accessibility & Inclusivity

Templates Status:
✅ spec-template.md - Reviewed, aligns with User Scenarios & Testing approach
✅ plan-template.md - Reviewed, aligns with Constitution Check section
✅ tasks-template.md - Reviewed, aligns with user story-driven approach
⚠ Commands in .specify/templates/commands/ - May need review to ensure Docusaurus-specific guidance

Follow-up TODOs:
- None at this time; all placeholders filled

Notes:
- This is the initial constitution for the Physical AI & Humanoid Robotics textbook project
- Major version 1.0.0 reflects establishment of core governance principles
- All principles are testable and align with educational content development standards
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Educational Clarity

**Purpose**: Ensure all content is accessible to learners from beginner to intermediate levels.

**Non-Negotiable Rules**:
- Every chapter MUST begin with clear learning objectives stating what students will know/be able to do
- Complex concepts MUST be introduced with real-world analogies before technical definitions
- Technical jargon MUST be defined on first use with glossary links
- Explanations MUST progress from simple to complex using the "scaffold" approach
- Each section MUST answer "Why does this matter?" before diving into "How does it work?"

**Rationale**: Students learning Physical AI and Humanoid Robotics come from diverse backgrounds. Breaking down complexity without sacrificing accuracy ensures maximum accessibility while maintaining rigor.

**Testing Criteria**:
- Can a reader with basic programming knowledge understand the introduction without external references?
- Are all new terms linked to glossary definitions?
- Does each chapter clearly state prerequisites?

---

### II. Structured Learning Progression

**Purpose**: Maintain consistent chapter organization that supports cognitive load management.

**Non-Negotiable Rules**:
- Every chapter MUST include these sections in order:
  1. **Introduction**: Context and motivation (what/why)
  2. **Core Concepts**: Foundational theory with diagrams
  3. **Practical Examples**: Hands-on code walkthroughs
  4. **Exercises**: Graduated difficulty (basic → intermediate → challenge)
  5. **Summary**: Key takeaways and next steps
- Chapters MUST build on prior knowledge sequentially (Module 1 → 2 → 3 → 4)
- Cross-references to earlier chapters MUST be explicit with navigation links
- Prerequisites MUST be stated at chapter start

**File Naming Convention**:
- Chapters: `docs/module-[N]/[##]-[topic-name].md`
- Assets: `static/img/module-[N]/[chapter-##]/[descriptive-name].[ext]`
- Code examples: `static/code/module-[N]/[chapter-##]/[example-name].[py|cpp]`

**Rationale**: Cognitive science research shows structured progression reduces learning friction. Predictable organization lets students focus on content, not navigation.

**Testing Criteria**:
- Does the chapter follow the 5-section template?
- Can each chapter be understood if prerequisites are met?
- Are file paths following naming conventions?

---

### III. Code Quality & Reproducibility

**Purpose**: Ensure all code examples are executable, well-documented, and follow best practices.

**Non-Negotiable Rules**:
- All code examples MUST be tested and executable in the stated environment
- Code blocks MUST include:
  - Language identifier for syntax highlighting
  - Inline comments explaining non-obvious logic
  - Expected output or behavior description
- Python code MUST follow PEP 8 style guidelines
- C++ code MUST follow Google C++ Style Guide
- Environment setup (ROS 2 version, dependencies) MUST be specified in each module introduction
- Code examples MUST include error handling for common failure modes

**Example Code Block Format**:
````markdown
```python title="src/example.py"
# Initialize ROS 2 node for robot control
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Robot controller initialized')

# Expected output: [INFO] [robot_controller]: Robot controller initialized
```
````

**Rationale**: Students must be able to run examples successfully to build confidence. Reproducibility is fundamental to scientific and engineering education.

**Testing Criteria**:
- Can code be copy-pasted and executed without modification?
- Are dependencies and versions clearly documented?
- Do error messages match expected outputs?

---

### IV. Visual & Multimodal Learning

**Purpose**: Support diverse learning styles through strategic use of diagrams, animations, and visuals.

**Non-Negotiable Rules**:
- Every abstract concept (e.g., coordinate frames, sensor fusion) MUST have a visual diagram
- Diagrams MUST be created using:
  - Mermaid.js for flowcharts, sequence diagrams, state machines
  - SVG/PNG for architectural diagrams (with source files in `/static/diagrams-source/`)
- Images MUST include:
  - Descriptive alt text for accessibility
  - Figure captions explaining what the visual represents
  - Source attribution if not original
- Visual complexity MUST match section complexity (simple visuals early, detailed later)
- Animations or videos SHOULD be used for:
  - Robot motion demonstrations
  - Simulation walkthroughs
  - Step-by-step processes

**Image Standards**:
- Format: PNG for screenshots, SVG for diagrams, GIF/MP4 for animations
- Naming: `[module]-[chapter]-[concept]-[variant].ext`
- Max width: 800px for inline, 1200px for full-width
- Compression: Optimize for web (use ImageOptim or similar)

**Rationale**: Visual processing is faster than text for many learners. Diagrams reduce cognitive load when explaining spatial relationships (critical in robotics).

**Testing Criteria**:
- Does each complex concept have a corresponding visual?
- Are all images accessible (alt text present)?
- Are diagrams editable (source files preserved)?

---

### V. Interactive Engagement

**Purpose**: Transform passive reading into active learning through exercises and interactive elements.

**Non-Negotiable Rules**:
- Each chapter MUST include 3+ exercises:
  - **Basic**: Apply a single concept directly (e.g., "Modify this ROS 2 node to publish at 10Hz")
  - **Intermediate**: Combine 2-3 concepts (e.g., "Create a node that subscribes to sensor data and publishes filtered output")
  - **Challenge**: Open-ended problem requiring synthesis (e.g., "Design a navigation stack for a quadruped robot")
- Hands-on projects MUST:
  - Include step-by-step instructions
  - Specify expected outcomes
  - Provide starter code templates
  - Offer extension challenges for advanced learners
- AI chatbot integration points MUST:
  - Appear at end of sections for "Ask questions about this topic"
  - Link to specific context (e.g., "Ask about ROS 2 parameter servers")
  - Suggest common questions students might ask

**Exercise Format**:
```markdown
### Exercise 2.3: Sensor Data Filtering (Intermediate)

**Objective**: Apply moving average filtering to IMU data in ROS 2.

**Instructions**:
1. Create a subscriber node for `/imu/data`
2. Implement a 10-sample moving average filter
3. Publish filtered data to `/imu/filtered`

**Starter Code**: [Link to template]

**Expected Outcome**: Filtered IMU readings with reduced noise variance.

**Extension**: Try implementing a Kalman filter instead.
```

**Rationale**: Active recall and application solidify learning better than passive reading. Graduated difficulty builds confidence while challenging advanced students.

**Testing Criteria**:
- Does each chapter have 3+ exercises with clear difficulty levels?
- Are starter code templates provided for intermediate+ exercises?
- Are chatbot prompts contextually relevant?

---

### VI. Technical Accuracy & Currency

**Purpose**: Maintain correctness and relevance in a rapidly evolving field.

**Non-Negotiable Rules**:
- All technical claims MUST be verifiable through:
  - Official documentation links (ROS 2, NVIDIA Isaac, Unity)
  - Peer-reviewed papers (for theoretical concepts)
  - Industry standards (IEEE, ISO robotics standards)
- Version-specific information MUST include:
  - Software version numbers (e.g., "ROS 2 Humble", "Isaac Sim 2023.1")
  - Deprecation notices if using older APIs
  - Migration guides when breaking changes occur
- Content reviews MUST occur:
  - Before major releases (1.0, 2.0)
  - Annually for version updates
  - When major framework updates are released (e.g., new ROS 2 LTS)
- Corrections MUST be tracked in a changelog with:
  - Date of correction
  - Nature of error (factual, code, conceptual)
  - Sections affected

**Update Policy**:
- **Critical errors** (broken code, incorrect physics): Immediate fix + announcement
- **Minor errors** (typos, formatting): Batched in monthly updates
- **Framework updates**: Quarterly review of major dependencies

**Rationale**: Robotics frameworks evolve rapidly. Outdated information undermines trust and leads to student frustration when examples fail.

**Testing Criteria**:
- Are all external references current and accessible?
- Are version numbers specified for all tools/frameworks?
- Is there a changelog tracking content updates?

---

### VII. Accessibility & Inclusivity

**Purpose**: Ensure content is usable by learners with diverse abilities, backgrounds, and resources.

**Non-Negotiable Rules**:
- Language clarity:
  - MUST use clear, direct language (avoid idioms, culturally-specific references)
  - MUST define acronyms on first use in each chapter
  - SHOULD use active voice and present tense
  - MUST avoid gendered language (use "they/them" or role titles)
- Visual accessibility:
  - All images MUST have descriptive alt text
  - Color MUST NOT be the only means of conveying information (use patterns/labels too)
  - Text contrast MUST meet WCAG 2.1 AA standards (4.5:1 for normal text)
- Hardware considerations:
  - Simulations MUST be provided as alternatives to physical robots
  - Resource requirements MUST be stated (e.g., "Requires GPU with 4GB VRAM")
  - Cloud/free alternatives MUST be mentioned when possible (e.g., Google Colab for GPU access)
- Skill level support:
  - Beginner content MUST NOT assume prior robotics knowledge
  - Advanced sections MUST be clearly marked as optional
  - Multiple explanation styles SHOULD be provided (mathematical, visual, code-based)

**Example - Multiple Explanation Styles**:
```markdown
### Understanding Coordinate Transformations

**Intuitive**: Imagine you're giving directions. "Turn left" depends on which way you're facing.
Coordinate transformations tell robots how to translate "left" in their frame to "left" in the
world's frame.

**Mathematical**: A transformation matrix T converts coordinates from frame A to frame B:
P_B = T_{AB} × P_A

**Code-Based**: In ROS 2, use tf2 to transform a point:
[code example]
```

**Rationale**: Diverse learners have diverse needs. An accessible textbook reaches the widest audience and upholds equity in robotics education.

**Testing Criteria**:
- Can screen readers navigate the content effectively?
- Are hardware requirements documented upfront?
- Are simulation alternatives available for all hands-on content?

---

## Content Development Standards

### Writing Style

- **Tone**: Professional yet conversational, encouraging, beginner-friendly
- **Voice**: Second person ("you will learn") for engagement
- **Sentence length**: Vary length; aim for 15-20 words average
- **Paragraph length**: 3-5 sentences; use whitespace liberally
- **Examples**: Use realistic robotics scenarios (warehouse robots, assistive devices, humanoid navigation)

### Chapter Templates

Every chapter MUST follow this Markdown template structure:

```markdown
---
sidebar_position: [number]
title: "[Chapter Title]"
description: "[One-sentence chapter description]"
---

# [Chapter Title]

## Learning Objectives
By the end of this chapter, you will:
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Prerequisites
- [Prerequisite knowledge/chapters]

---

## Introduction
[Why this topic matters, real-world context]

## Core Concepts
[Foundational theory with diagrams]

## Practical Examples
[Hands-on code walkthroughs]

## Exercises
[3+ graduated exercises]

## Summary
[Key takeaways, next steps]

## Further Reading
[Optional: Links to advanced resources]
```

### Docusaurus Integration

- Sidebar organization in `sidebars.ts` MUST reflect module structure
- Front matter MUST include `sidebar_position` and `title`
- Code blocks MUST use language identifiers: `python`, `cpp`, `bash`, `yaml`
- Admonitions SHOULD be used for:
  - `:::tip` for helpful shortcuts
  - `:::warning` for common pitfalls
  - `:::note` for additional context
  - `:::danger` for critical safety/security concerns

---

## Quality Assurance

### Peer Review Process

- **Draft Review**: Technical SME reviews for accuracy
- **Educational Review**: Learning designer reviews for pedagogy
- **Accessibility Review**: Check WCAG compliance + screen reader testing
- **Student Beta Test**: 3-5 target audience members test chapter independently

### Consistency Checks (Pre-Publish)

- [ ] All code examples tested in stated environment
- [ ] All images have alt text
- [ ] All chapters follow 5-section template
- [ ] All cross-references resolve correctly
- [ ] All external links are valid (automated check)
- [ ] Glossary terms are defined and linked
- [ ] File naming conventions followed
- [ ] Mermaid diagrams render correctly

### Maintenance Schedule

- **Monthly**: Fix reported errors, update broken links
- **Quarterly**: Review major framework version updates (ROS 2, Isaac, Unity)
- **Annually**: Comprehensive technical accuracy review
- **As needed**: Respond to student feedback, add clarifications

---

## AI Chatbot Integration

### Embedding Strategy

- Chatbot widget appears in bottom-right corner on all content pages
- Context-aware prompts at section ends (e.g., "Ask me about sensor fusion algorithms")
- RAG (Retrieval-Augmented Generation) trained on:
  - All textbook content
  - Official documentation (ROS 2, Gazebo, Isaac)
  - Curated Q&A from student feedback

### Chatbot Constraints

- MUST provide sources for all factual claims
- MUST NOT provide solutions to exercises directly (guide with hints instead)
- MUST redirect to relevant textbook sections
- SHOULD ask clarifying questions before answering
- MUST include disclaimer: "AI-generated responses may contain errors; verify critical information"

---

## Module-Specific Standards

### Module 1: ROS 2 Fundamentals
- Focus on publisher/subscriber patterns, services, parameters
- Examples MUST use ROS 2 Humble (LTS) or newer
- Simulation: Turtlesim for basic examples, Gazebo for advanced

### Module 2: Simulation (Gazebo & Unity)
- Gazebo examples MUST use Gazebo Fortress or Garden
- Unity examples MUST use Unity Robotics Hub package
- MUST compare when to use Gazebo (physics-focused) vs Unity (sensor/ML-focused)

### Module 3: NVIDIA Isaac
- Examples MUST specify Isaac Sim vs Isaac Gym context
- GPU requirements MUST be stated upfront
- Cloud alternatives (e.g., Omniverse Cloud) MUST be mentioned

### Module 4: Vision-Language-Action (VLA)
- ML models MUST include:
  - Pre-trained model sources
  - Dataset requirements
  - Training time estimates
  - Inference performance metrics
- Ethics section MUST address:
  - Bias in training data
  - Safety considerations for embodied AI
  - Responsible deployment practices

---

## Governance

### Authority

This constitution supersedes all other project documentation for content development decisions. When conflicts arise between this constitution and other guidelines, the constitution takes precedence.

### Amendment Process

1. **Proposal**: Submit amendment via GitHub issue with:
   - Rationale (why change is needed)
   - Impact analysis (what content will be affected)
   - Migration plan (how to update existing content)
2. **Review**: Core team reviews proposal within 2 weeks
3. **Approval**: Requires consensus from:
   - Technical lead (accuracy)
   - Educational lead (pedagogy)
   - Accessibility lead (inclusivity)
4. **Implementation**:
   - Update constitution version (semantic versioning)
   - Update affected templates
   - Create migration checklist for existing content
   - Announce changes to contributors

### Versioning Rules

- **MAJOR** (X.0.0): Removal or redefinition of core principles (backward incompatible)
- **MINOR** (0.X.0): New principle added, material expansion of existing principle
- **PATCH** (0.0.X): Clarifications, typo fixes, non-semantic refinements

### Compliance Verification

- All pull requests MUST include a constitution checklist:
  - [ ] Learning objectives stated
  - [ ] 5-section structure followed
  - [ ] Code tested and executable
  - [ ] Images have alt text
  - [ ] Exercises included (3+ levels)
  - [ ] Version numbers specified for tools
  - [ ] File naming conventions followed
- Automated checks SHOULD verify:
  - Markdown linting
  - Link validation
  - Image alt text presence
  - Code block language identifiers
- Manual review gate for principle compliance before merge

### Exception Handling

Exceptions to constitutional principles MUST be:
- Documented in PR description with justification
- Approved by 2+ core team members
- Logged in `docs/constitutional-exceptions.md` with:
  - Date and PR number
  - Principle(s) violated
  - Justification
  - Mitigation plan (if applicable)

---

**Version**: 1.0.0 | **Ratified**: 2025-11-28 | **Last Amended**: 2025-11-28
