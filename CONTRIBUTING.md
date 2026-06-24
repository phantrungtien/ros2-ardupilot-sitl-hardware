# Contributing to ROS2-ArduPilot SITL & Hardware Integration

Thank you for your interest in contributing! This project helps researchers, students, and developers work with autonomous drones using ROS2 and ArduPilot.

## 🎯 How You Can Contribute

### 1. **Report Bugs**
- Use the [GitHub Issues](https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/issues) page
- Include your environment details (ROS2 version, hardware, OS)
- Provide steps to reproduce the issue
- Include relevant logs and error messages

### 2. **Suggest Enhancements**
- Open an issue with the tag `enhancement`
- Clearly describe the feature and its benefits
- Explain use cases

### 3. **Submit Code**
- Fork the repository
- Create a feature branch (`git checkout -b feature/amazing-feature`)
- Make your changes
- Test thoroughly (both SITL and hardware if possible)
- Commit with clear messages
- Push to your fork
- Open a Pull Request

### 4. **Improve Documentation**
- Fix typos or clarify confusing sections
- Add examples or tutorials
- Translate documentation (future)

### 5. **Share Your Experience**
- Tested on different hardware? Let us know!
- Created interesting missions? Share them!
- Found better configurations? Submit them!

---

## Development Guidelines

### Code Style
- Follow ROS2 Python style guidelines
- Use meaningful variable names
- Add comments for complex logic
- Keep functions focused and simple

### Testing Requirements
- Test in SITL before submitting
- If possible, test on real hardware
- Verify existing functionality still works
- Include test results in PR description

### Commit Messages
Use clear, descriptive commit messages:
```
✅ Good:
- "Add GPS waypoint validation in mission script"
- "Fix MAVROS connection timeout in real hardware setup"
- "Update README with Jetson Nano compatibility"

❌ Bad:
- "fix bug"
- "update"
- "changes"
```

---

## Testing Your Changes

### Before Submitting:

1. **Clean Build Test:**
   ```bash
   cd ~/ros2-ardupilot-sitl-hardware
   rm -rf build/ install/ log/
   colcon build
   ```

2. **SITL Test:**
   ```bash
   ./launch/start_sitl.sh  # Terminal 1
   ./launch/start_mavros.sh  # Terminal 2
   source install/setup.bash && python3 scripts/missions/mission_simple.py  # Terminal 3
   ```

3. **Code Quality:**
   ```bash
   # Python linting
   flake8 src/
   ```

---

## Pull Request Process

1. **Update Documentation**
   - If you add features, update relevant .md files
   - Update CHANGELOG.md with your changes

2. **Ensure Builds Pass**
   - Clean build must succeed
   - No new warnings

3. **Describe Your Changes**
   - What does this PR do?
   - Why is this change needed?
   - What testing did you perform?
   - Any breaking changes?

4. **Wait for Review**
   - Maintainer will review within a week
   - Address feedback promptly
   - Be patient and respectful

---

## Safety Guidelines

When contributing features related to **real hardware**:

- ⚠️ **Always emphasize safety in documentation**
- ⚠️ **Include propeller removal reminders**
- ⚠️ **Test bench testing before flight**
- ⚠️ **Provide emergency procedures**
- ⚠️ **Never encourage unsafe practices**

---

## Getting Help

- **Questions:** Open a [Discussion](https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/discussions)
- **Bugs:** Open an [Issue](https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/issues)
- **Ideas:** Open an issue with `enhancement` tag

---

## Recognition

Contributors will be:
- Listed in the project README
- Credited in release notes
- Acknowledged in documentation

---

## Code of Conduct

### Our Standards

- Be respectful and inclusive
- Welcome newcomers
- Accept constructive criticism
- Focus on what's best for the community
- Show empathy towards others

### Unacceptable Behavior

- Harassment or discrimination
- Trolling or insulting comments
- Publishing others' private information
- Any conduct inappropriate in a professional setting

---

## Contact

**Maintainer:** phantrungtien  
**GitHub:** [@phantrungtien](https://github.com/phantrungtien)  
**Email:** phantrungtien1034@gmail.com

---

Thank you for making this project better! 🚁
