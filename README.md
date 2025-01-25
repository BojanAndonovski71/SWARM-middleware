# Robotic SWARM
Heterogeneous Underwater Code and Utility

Project Purpose
SWARM Middleware is a robust framework designed to facilitate communication and coordination among SWARM robots. It provides a scalable, efficient, and modular middleware to enable distributed decision-making, resource-sharing, and fault-tolerant operations for SWARM systems.

Features
Distributed Communication: Enables message passing and data synchronization across SWARM robots.
Task Allocation: Dynamic task assignment based on robot capabilities.
Fault Tolerance: Resilient to node failures with fallback mechanisms.
Modularity: Extensible architecture to integrate new sensors and actuators.

Project Structure
SWARM-middleware/
├── src/                # Source code for the middleware
├── docs/               # Documentation and technical details
├── examples/           # Example implementations
├── tests/              # Unit tests and integration tests
├── LICENSE             # License file
└── README.md           # Project documentation

Installation
Follow these steps to set up the SWARM Middleware:

Clone the Repository:
git clone https://github.com/BojanAndonovski71/SWARM-middleware.git
cd SWARM-middleware

Install Dependencies:
Ensure you have Python 3.8+ installed.
Install required dependencies:
pip install -r requirements.txt

Build the Middleware:
Use the provided build script:
./build.sh




