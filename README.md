# ME495 Demo
A demonstration of Basic ROS 2 concepts for [Northwestern University's ME495 Embedded Systems in Robotics](https://nu-msr.github.io/ros_notes)

1. The git history of this repository shows various milestones in the building of a ROS package.

2. Each step is tagged in git starting with `step0`

3. `step0` is the state of the repository after running
   `ros2 pkg create --build-type ament_python me495_demo`
   and then committing the results.
4. You can explore each step as follows (replace `<X>` with the step number).
   ```
   git checkout step<X>
   git diff --word-diff=color step<X-1>
   ```

5. View these instructions anytime with `git show main:README.md`
