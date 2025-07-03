# LeRobot Project

This repository contains a comprehensive robotics development environment with multiple components for building, simulating, and controlling robots.

## 🚀 Project Overview

This project consists of several interconnected components:

- **🤖 LeRobot Library**: A state-of-the-art AI robotics library for real-world robotics
- **🌐 LeLab Space**: Web-based robot control and visualization interface
- **🎮 SO100 MuJoCo Sim**: MuJoCo-based robot simulation environment
- **📷 Camera Tools**: Various camera testing and control utilities

## 📁 Project Structure

```
lerobot/
├── lerobot/                 # Main LeRobot Python library
│   ├── examples/           # Tutorial and example scripts
│   ├── lerobot/           # Core library code
│   ├── tests/             # Test suite
│   ├── docs/              # Documentation
│   └── README.md          # Detailed LeRobot documentation
├── leLab-space/           # Web application for robot control
│   ├── src/               # Frontend source code
│   ├── public/            # Static assets
│   └── README.md          # Web app documentation
├── so100-mujoco-sim/      # MuJoCo simulation environment
│   ├── src/               # Simulation source code
│   ├── scripts/           # Simulation utilities
│   └── README.md          # Simulation documentation
├── *.py                   # Camera testing and control scripts
└── README.md              # This file
```

## 🛠️ Quick Start

### 1. LeRobot Library

The main robotics library provides state-of-the-art AI models, datasets, and tools for real-world robotics.

```bash
cd lerobot
pip install -e .
```

For detailed installation and usage instructions, see [lerobot/README.md](lerobot/README.md).

### 2. LeLab Space (Web Interface)

A modern web application for robot control and visualization.

```bash
cd leLab-space
npm install
npm run dev
```

For more details, see [leLab-space/README.md](leLab-space/README.md).

### 3. SO100 MuJoCo Simulation

MuJoCo-based simulation environment for robot testing and development.

```bash
cd so100-mujoco-sim
# Follow the instructions in so100-mujoco-sim/README.md
```

### 4. Camera Tools

Various Python scripts for camera testing and control:

- `test_camera.py` - Basic camera testing
- `use_camera.py` - Camera usage examples
- `list_cameras.py` - List available cameras
- `find_cameras.py` - Find and configure cameras

## 🎯 Key Features

### LeRobot Library
- 🤖 State-of-the-art AI models for robotics
- 📊 Pre-trained models and datasets
- 🎮 Simulation environments (ALOHA, PushT, XArm)
- 🔧 Real-world robot control capabilities
- 📚 Comprehensive documentation and examples

### LeLab Space
- 🌐 Modern web interface
- 📱 Responsive design
- 🎮 Real-time robot control
- 📊 Data visualization
- 🔧 Configuration management

### SO100 MuJoCo Sim
- 🎮 High-fidelity robot simulation
- 🔧 Customizable environments
- 📊 Performance benchmarking
- 🧪 Research and development tools

## 📚 Documentation

- [LeRobot Library Documentation](lerobot/README.md) - Complete guide to the robotics library
- [LeLab Space Documentation](leLab-space/README.md) - Web interface setup and usage
- [SO100 MuJoCo Sim Documentation](so100-mujoco-sim/README.md) - Simulation environment guide
- [LeRobot Official Docs](https://huggingface.co/docs/lerobot) - Official documentation

## 🤝 Contributing

We welcome contributions! Please see the individual component README files for contribution guidelines:

- [LeRobot Contributing Guide](lerobot/CONTRIBUTING.md)
- [Code of Conduct](lerobot/CODE_OF_CONDUCT.md)

## 📄 License

This project is licensed under the Apache 2.0 License. See the [LICENSE](lerobot/LICENSE) file for details.

## 🆘 Support

- 📖 [Documentation](https://huggingface.co/docs/lerobot)
- 💬 [Discord Community](https://discord.gg/s3KuuzsPFb)
- 🐛 [GitHub Issues](https://github.com/huggingface/lerobot/issues)
- 📧 [Email Support](mailto:lerobot@huggingface.co)

## 🙏 Acknowledgments

This project builds upon the work of many researchers and developers in the robotics and AI communities. Special thanks to:

- The Hugging Face team for the LeRobot library
- The MuJoCo development team
- All contributors and community members

---

**Happy Robotics Development! 🤖✨** 