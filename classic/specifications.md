You are helping me create detailed SPECIFICATIONS for a "Physical AI & Humanoid Robotics" textbook based on the constitution we established.

## Context:
This is an interactive online textbook built with Docusaurus that teaches Physical AI and Humanoid Robotics. It includes 4 main modules with embedded chatbot, code examples, and hands-on exercises.

## Course Content to Cover:

### Module 1: The Robotic Nervous System (ROS 2)
- ROS 2 Nodes, Topics, and Services
- Bridging Python Agents to ROS controllers using rclpy
- Understanding URDF (Unified Robot Description Format) for humanoids

### Module 2: The Digital Twin (Gazebo & Unity)
- Simulating physics, gravity, and collisions in Gazebo
- High-fidelity rendering and human-robot interaction in Unity
- Simulating sensors: LiDAR, Depth Cameras, and IMUs

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation
- Isaac ROS: Hardware-accelerated VSLAM and navigation
- Nav2: Path planning for bipedal humanoid movement

### Module 4: Vision-Language-Action (VLA)
- Voice-to-Action: Using OpenAI Whisper for voice commands
- Cognitive Planning: Using LLMs to translate natural language into ROS 2 actions
- Capstone Project: Autonomous Humanoid

## Create DETAILED SPECIFICATIONS for:

### 1. Overall Book Structure
- Total number of chapters per module
- Chapter naming convention
- Navigation/sidebar organization
- Homepage and introduction structure

### 2. Each Module Specification
For each of the 4 modules, define:
- Module overview and learning objectives
- List of chapters with brief description
- Prerequisites for that module
- Estimated completion time
- Key deliverables/projects

### 3. Chapter Template Specification
Define the standard structure every chapter must follow:
- Required sections (intro, concepts, examples, exercises, summary)
- Word count ranges for each section
- Number of code examples per chapter
- Number of diagrams/visuals required
- Exercise difficulty progression

### 4. Technical Content Specification
- Programming languages and versions (Python 3.10+, ROS 2 Humble, etc.)
- Code example complexity levels
- Hardware/software requirements to mention
- External resources and references format

### 5. Interactive Features Specification
- RAG Chatbot capabilities and placement
- Exercise types (multiple choice, coding challenges, projects)
- Code playground requirements
- User interaction points

### 6. Visual Content Specification
- Types of diagrams needed (architecture, flowcharts, system diagrams)
- Mermaid diagram requirements
- Image/screenshot specifications
- Video embedding guidelines (if any)

### 7. Assessment Specification
- Quiz questions per module
- Hands-on project requirements
- Capstone project detailed specification
- Success criteria for each assessment

### 8. Documentation Standards
- Markdown formatting rules
- Frontmatter requirements for each file
- Metadata standards (tags, categories, authors)
- Version control and file naming

### 9. Deployment Specification
- GitHub Pages configuration
- Build and deployment process
- Environment variables needed
- CI/CD pipeline requirements

### 10. Chatbot Integration Specification
- FastAPI backend endpoints required
- Vector database structure (Qdrant)
- User database schema (Neon Postgres)
- Authentication requirements (Better-auth)
- Personalization features specification
- Urdu translation implementation

Think step by step and create comprehensive, actionable specifications that any developer can follow to build this textbook. Be specific about numbers, formats, and requirements.

Output the specifications in a well-organized markdown document with clear sections and subsections.