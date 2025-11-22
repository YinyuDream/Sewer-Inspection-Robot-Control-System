# Mid-term Check Report

## Project Summary
This project aims to build a control core for a sewer inspection robot using a high-performance microcontroller, integrating subsystems including locomotion control, power management, and sensor data acquisition. Key topics include the design of a robust communication architecture, a multi-task scheduling program for real-time performance, pulse-width modulation motor control implementation, and the development of sensor data synchronization interfaces. The final outcome is a fully functional, stable, and logically complete embedded control prototype capable of operating in demanding environments.

## Expected Goals
(1) Developing a multi-task scheduling program to support parallel task execution and real-time response, ensuring critical control loops meet timing deadlines.
(2) Completing the design of a motor control module to achieve precise speed and torque control, utilizing advanced timer peripherals.
(3) Building interfaces to receive processed sensor data from the navigation system. Note that while I require sensor integration, the complex data processing (e.g., SLAM) is handled by a separate navigation module; my focus is strictly on the control system acting upon this processed data.
(4) Validating the control system through high-fidelity simulation and hardware-in-the-loop experiments, resulting in a stable and fully functional embedded control prototype. The final outcome will lay the core control foundation for subsequent robot prototyping and engineering applications.

## Current Progress and Achievements

### 1. Project Aims and Objectives
The primary aim of this project is to design and implement a robust control system for a sewer inspection robot. Sewer environments present unique challenges, including confined spaces, slippery surfaces, and the need for high reliability. Therefore, the control system must be autonomous, robust against disturbances, and capable of precise actuation. The specific objectives for this phase were to establish a high-fidelity simulation environment to validate control algorithms before hardware deployment, and to design the software architecture based on a modern robot operating system framework.

The project adopts a Model-Based Design approach. By simulating the physical dynamics of the robot and the environment, I can iterate on control strategies rapidly without the risk of damaging expensive hardware. The ultimate goal is to port these validated algorithms to an embedded platform, ensuring real-time performance and deterministic behavior.

2. Literature Review
The field of sewer inspection robotics has evolved significantly over the past decades. Early systems were tethered, teleoperated platforms with limited autonomy. Recent literature emphasizes the shift towards fully autonomous systems capable of intelligent perception and decision-making.

Regarding locomotion, research suggests that tracked vehicles offer superior traction and stability compared to wheeled platforms for pipe inspection [1]. While skid-steer wheeled platforms are mechanically simple, tracks provide a larger contact area, which is crucial for navigating slippery and uneven sewer surfaces. However, tracked skid-steering introduces non-holonomic constraints and complex track-terrain interaction, especially in curved pipes, requiring sophisticated control strategies [2].
In terms of control architectures, the adoption of modular middleware in field robotics is well-documented. Studies demonstrate the modularity of such systems, which allows for easy integration of disparate sensors [3]. For embedded control, lightweight frameworks are gaining traction, allowing microcontrollers to participate directly in the distributed control graph [4].
For simulation, the use of physics engines for simulating underground environments is a standard practice. Literature highlights the importance of physics engines in validating robot interaction with complex geometries [5]. Recent works have focused on procedural generation of environments to test robots against a wide variety of pipe topologies.

**References:**
[1] Roslin, N. S., et al. "A Review: Hybrid Locomotion of In-Pipe Inspection Robot." *Procedia Engineering*, vol. 41, 2012, pp. 1456-1462.
[2] Yi, J., et al. "Kinematic Modeling and Analysis of Skid-Steered Mobile Robots with Applications to Low-Cost Inertial-Measurement-Unit-Based Motion Estimation." *IEEE Transactions on Robotics*, vol. 25, no. 5, 2009, pp. 1087-1097.
[3] Quigley, M., et al. "ROS: an open-source Robot Operating System." *ICRA workshop on open source software*, vol. 3, no. 3.2, 2009.
[4] Strunck, J., et al. "Micro-ROS: A Real-Time Robot Operating System for Microcontrollers." *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2020.
[5] Koenig, N., and Howard, A. "Design and use of Gazebo, an open-source 3D multi-robot simulator." *2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, vol. 3, 2004, pp. 2149-2154.

3. Background Information
The choice of a modern robot operating system and a high-fidelity physics simulator forms the backbone of this project's software infrastructure. The operating system offers real-time capabilities and improved reliability, which are critical for an industrial inspection robot. The simulation is built upon a bridge architecture, which facilitates seamless communication between the control logic and the physics simulation.

The currently simulated robot is a four-wheel independent drive system, serving as a kinematic prototype for the future tracked design. This configuration allows for differential steering, which is mechanically simple and robust—ideal for the harsh conditions of a sewer. The generated pipe environment represents a preliminary prototype, sufficient for testing control logic but distinct from the complex geometry of the final deployment site. The control logic currently resides in high-level scripts for rapid prototyping, with the intention to migrate time-critical components to compiled languages and eventually to the microcontroller firmware.

4. Progress to Date and Outcomes

A. Simulation Environment Development
A significant achievement in this phase is the development of a procedural generation tool for sewer environments. A specialized script was developed to mathematically generate three-dimensional pipe models. Unlike static mesh imports, this tool calculates the geometry of straight sections and corners, stitching them together into a continuous loop.
The tool utilizes numerical libraries to compute the reference frames along a path. It generates vertices and normals for a cylindrical pipe profile, ensuring that the normals point inward to correctly simulate the interior of a sewer. A complex pipe network model is generated and integrated into the simulation world. This provides a realistic testing ground for the robot's interaction with the environment.

B. Robot Modeling and Kinematics
The robot has been fully modeled using a unified robot description format. The model defines the physical properties of the robot:
The chassis is modeled as a rigid body with appropriate inertial properties to ensure realistic dynamic behavior during acceleration and collisions.
The drive train consists of four wheels modeled as cylinders with high-friction surfaces to simulate rubber tires on concrete. The joints are continuous, driven by a joint controller plugin.
An inertial measurement unit is mounted on the chassis. The simulation plugin is configured to publish linear acceleration and angular velocity data at a high frequency. This is crucial for future sensor fusion tasks.
A ground truth publisher provides accurate position data for validating state estimation algorithms.

C. Control System Implementation
The software control stack has been implemented and tested, leveraging the modularity of the operating system.
The system startup configuration orchestrates the initialization of the simulation, the robot state publisher, the communication bridge, and the visualization tools. This ensures a streamlined workflow for the development environment. It uses inclusion mechanisms to wrap standard simulation launch files, passing specific arguments to load the custom world.
The communication bridge has been configured to map simulation topics to control system topics. Specifically, it bridges the velocity command signals for individual wheels and the sensor data streams. This bridge is critical as it translates the data distribution service messages used by the control system into the transport layer used by the simulator.
A custom manual control node was developed using the client library. Unlike standard teleoperation tools, this node maps input signals directly to differential wheel velocities.
The logic initializes publishers for each wheel controller topic. It captures input signals in raw mode to ensure immediate response.
It implements a kinematic model where inputs increment or decrement the linear velocity target for all wheels, and introduce a differential between left and right wheel pairs to induce rotation. This validates the differential drive mixing logic before it is implemented on the microcontroller.
The node includes a safety feature to immediately zero out all velocities, a crucial feature for testing in confined pipe environments.
The robot successfully moves within the generated pipe world under manual control. The differential steering mechanic functions as expected, allowing the robot to turn in place and maneuver through the environment.

D. Visualization and Debugging
Visualization tools have been integrated into the workflow. The configuration is set up to visualize the robot model, the coordinate frames, and the sensor data. This allows for real-time debugging of the kinematic chain and sensor placement. I can visually verify that the wheel transforms are correctly updating as the robot moves, confirming that the joint state publisher is functioning correctly.

E. Procedural Environment Generation Details
The procedural generation module represents a significant software asset. It addresses the challenge of creating varied test environments without manual modeling.
The algorithm defines a path using a sequence of straight lines and circular arcs. It discretizes these segments into points and calculates the tangent vector at each point.
For every point on the path, it generates a circle of vertices perpendicular to the tangent. These vertices are then triangulated to form the pipe surface.
The module automatically handles the connection between the last and first points to ensure a watertight mesh, which is essential for the physics engine to detect collisions correctly.

In summary, the foundational work for the control system is complete. The simulation environment is operational, the robot model is physically accurate, and the basic control loop is closed. This sets the stage for implementing advanced autonomous features.

## Existing Problems and Proposed Solutions

Despite the progress, several challenges have been identified during the simulation and initial testing phases.

1. Control Challenges on Curved Surfaces
The current differential drive model assumes a flat surface. In a cylindrical pipe, the contact points of the wheels change as the robot moves up the walls. This alters the effective track width and causes unpredictable slippage, making precise open-loop control difficult.
I will implement a closed-loop control system. By using the inertial sensor data, specifically the gyroscope, I can implement a feedback controller that corrects the wheel velocities to maintain stability. Furthermore, I will investigate kinematic modeling for skid-steer vehicles on curved surfaces to refine the control inputs.

2. Sensor Signal Quality
While the simulation currently provides ground truth, the simulated inertial sensor includes noise. Preliminary tests show that integrating the raw accelerometer data leads to significant state drift within seconds.
A state estimation filter will be implemented using localization packages. This will fuse the inertial data with wheel odometry calculated from joint states. In the future hardware implementation, I will also integrate additional perception sensors to provide absolute corrections.

3. System Latency
There is a noticeable latency between the input and the robot's response in the simulation. This is due to the overhead of the communication bridge and the interpreted language execution speed.
For the simulation, I will optimize the quality of service settings to reduce latency. For the final embedded system, this problem will be mitigated by running the control loop directly on the microcontroller in a compiled language, eliminating the overhead of the operating system and the bridge.

4. Perception Limitations
The robot currently operates without environmental awareness. It cannot detect the pipe geometry or obstacles autonomously.
The next phase involves adding simulated ranging sensors to the robot model. I will then develop perception algorithms that use distance data to understand the robot's position relative to the pipe walls.

## Next Steps

The plan for the next phase of the project is divided into three main tracks: Advanced Control, Embedded Software Development, and Hardware Integration.

1. Advanced Control Strategies (Months 1-2)
The immediate focus is to transition from manual control to automated stability control within the simulation.
I will implement a feedback-based stability algorithm. Using simulated ranging sensors, the robot will adjust its steering to maintain a stable posture within the pipe.
I will develop a motion planning module that identifies the optimal path and generates velocity commands to follow it, ensuring smooth motion profiles.
I will tweak the friction coefficients in the robot description and the world model to better match the expected conditions of a real sewer, such as wet concrete or plastic.

2. Embedded System Migration (Months 3-4)
Once the control logic is validated in the simulation, I will begin porting the core algorithms to the embedded platform. This is a critical step to meet the project's real-time requirements.
I will set up a real-time operating system on the microcontroller. This corresponds to the project goal of developing a multi-task scheduling program. Tasks will be divided into:
Sensor Acquisition Task: High-priority task to read inertial and encoder data via direct memory access to minimize processor load.
Control Loop Task: Periodic task to execute the feedback or kinematic mixing algorithms.
Communication Task: Lower priority task to handle bus messages and debugging interfaces.
I will implement the low-level pulse-width modulation generation for the motor drivers. This involves configuring the microcontroller's advanced control timers to generate complementary signals with dead-time insertion to prevent hardware damage.
I will implement the controller area network communication stack. This will allow the main controller to communicate with the motor drivers and the sensor modules reliably. I will also implement a serial bridge to allow the embedded system to talk to the high-level computer, enabling the robot to receive commands directly from the microcontroller.

3. Sensor Fusion and State Estimation (Months 4-5)
I will develop the low-level drivers for the real inertial sensor on the microcontroller. I will implement a hardware abstraction layer to decouple the sensor logic from the specific hardware pins.
I will implement the sensor data synchronization interfaces mentioned in the project goals. This ensures that inertial data and wheel encoder data are time-stamped and aligned before being processed by the fusion algorithm. I will use a hardware timer as a common time base.
I will attempt to run a simplified state estimation filter on the microcontroller to estimate the robot's orientation in real-time, which is critical for stability in a pipe. This will involve optimizing mathematical operations for the processor architecture.

4. Hardware-in-the-Loop Testing (Month 6)
Before the final robot assembly, I will perform hardware-in-the-loop testing. The microcontroller board will be connected to the simulation computer. The simulation will send virtual sensor data to the microcontroller, and the microcontroller will send motor commands back to the simulation. This validates the embedded code without risking the physical robot.

## Evaluation of Completion
Based on the current progress, the project is well on track to be completed by the deadline of May 2026. The successful establishment of the simulation environment is a major milestone that mitigates significant technical risks. By validating the kinematic model and the control logic in software first, I have reduced the time required for hardware debugging.

The remaining tasks are clearly defined and follow a logical progression from high-level software to low-level firmware. The parallel development strategy—refining algorithms in the simulation while designing the embedded architecture—ensures that bottlenecks in one area do not stall the entire project. I am confident that the final prototype will meet all the specified requirements, including the multi-task scheduling and precise motor control, resulting in a high-quality engineering solution for sewer inspection.

---

# 中期检查报告

## 项目概述
本项目旨在利用高性能微控制器构建下水道检测机器人的控制核心，集成运动控制、电源管理和传感器数据采集等子系统。关键课题包括设计稳健的通信架构、用于实时性能的多任务调度程序、脉宽调制电机控制实现以及传感器数据同步接口的开发。最终成果将是一个功能齐全、稳定且逻辑完整的嵌入式控制原型，能够在苛刻的环境中运行。

## 预期目标
(1) 开发多任务调度程序以支持并行任务执行和实时响应，确保关键控制回路满足时序截止要求；
(2) 完成电机控制模块的设计，利用高级定时器外设实现精确的速度和扭矩控制；
(3) 构建接口以接收来自导航系统的处理后传感器数据。请注意，虽然我需要集成传感器，但复杂的数据处理（例如SLAM）由单独的导航模块处理；我的重点严格在于根据这些处理后的数据进行控制。
(4) 通过高保真仿真和硬件在环实验验证控制系统，从而产生稳定且功能齐全的嵌入式控制原型。最终成果将为后续的机器人原型设计和工程应用奠定核心控制基础。

## 目前进展及取得的成果

### 1. 项目目的和目标
本项目的主要目的是为下水道检测机器人设计并实现一个稳健的控制系统。下水道环境面临独特的挑战，包括受限空间、湿滑表面以及对高可靠性的需求。因此，控制系统必须具备自主性、抗干扰能力，并能够进行精确的驱动。本阶段的具体目标是建立一个高保真仿真环境，以便在硬件部署前验证控制算法，并基于现代机器人操作系统框架设计软件架构。

本项目采用基于模型的设计方法。通过模拟机器人和环境的物理动力学，我可以快速迭代控制策略，而无需冒损坏昂贵硬件的风险。最终目标是将这些经过验证的算法移植到嵌入式平台，确保实时性能和确定性行为。

2. 文献综述
下水道检测机器人领域在过去几十年中发生了显著变化。早期的系统是受限的、遥控的平台，自主性有限。最近的文献强调向具备智能感知和决策能力的完全自主系统转变。

关于运动方式，研究表明，履带式车辆在管道检测中比轮式平台提供更优越的牵引力和稳定性 [1]。虽然滑移转向轮式平台机械结构简单，但履带提供了更大的接触面积，这对于在湿滑和不平坦的下水道表面导航至关重要。然而，履带式滑移转向引入了非完整约束和复杂的履带-地形相互作用，特别是在弯曲管道中，需要复杂的控制策略 [2]。
在控制架构方面，模块化中间件在野外机器人中的应用已有充分记录。研究表明，此类系统的模块化允许轻松集成不同的传感器 [3]。对于嵌入式控制，轻量级框架正受到关注，允许微控制器直接参与分布式控制图 [4]。
在仿真方面，使用物理引擎模拟地下环境是一种标准做法。文献强调了物理引擎在验证机器人与复杂几何形状交互中的重要性 [5]。最近的工作集中在环境的过程化生成上，以测试机器人应对各种管道拓扑结构的能力。

**参考文献：**
[1] Roslin, N. S., et al. "A Review: Hybrid Locomotion of In-Pipe Inspection Robot." *Procedia Engineering*, vol. 41, 2012, pp. 1456-1462.
[2] Yi, J., et al. "Kinematic Modeling and Analysis of Skid-Steered Mobile Robots with Applications to Low-Cost Inertial-Measurement-Unit-Based Motion Estimation." *IEEE Transactions on Robotics*, vol. 25, no. 5, 2009, pp. 1087-1097.
[3] Quigley, M., et al. "ROS: an open-source Robot Operating System." *ICRA workshop on open source software*, vol. 3, no. 3.2, 2009.
[4] Strunck, J., et al. "Micro-ROS: A Real-Time Robot Operating System for Microcontrollers." *2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2020.
[5] Koenig, N., and Howard, A. "Design and use of Gazebo, an open-source 3D multi-robot simulator." *2004 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, vol. 3, 2004, pp. 2149-2154.

3. 背景信息
选择现代机器人操作系统和高保真物理模拟器构成了本项目软件基础设施的支柱。该操作系统提供实时功能和更高的可靠性，这对于工业检测机器人至关重要。仿真建立在桥接架构之上，促进了控制逻辑与物理仿真之间的无缝通信。

目前的仿真机器人采用轮式差速驱动作为原型，以验证控制逻辑，这与最终物理机器人的履带式设计有所不同。这种配置允许滑移转向，机械结构坚固——非常适合下水道的恶劣条件。生成的管道环境代表了一个初步的原型，足以测试控制逻辑，但与最终部署现场的复杂几何形状不同。目前的控制逻辑驻留在用于快速原型设计的高级脚本中，计划将时间关键型组件迁移到编译语言，并最终迁移到微控制器固件中。

4. 迄今为止的进展和成果

A. 仿真环境开发
本阶段的一个重大成就是开发了下水道环境的过程化生成工具。开发了一个专用脚本来数学生成三维管道模型。与静态网格导入不同，该工具计算直线段和拐角的几何形状，将它们拼接成一个连续的回路。
该工具利用数值库计算沿路径的参考帧。它生成圆柱形管道轮廓的顶点和法线，确保法线指向内部以正确模拟下水道的内部。生成了一个复杂的管道网络模型并将其集成到仿真世界中。这为机器人与环境的交互提供了逼真的测试场所。

B. 机器人建模与运动学
机器人已使用统一机器人描述格式进行了完整建模。该模型定义了机器人的物理属性：
底盘被建模为具有适当惯性属性的刚体，以确保加速和碰撞期间的逼真动态行为。
传动系统由四个车轮组成，建模为具有高摩擦表面的圆柱体，以模拟混凝土上的橡胶轮胎。关节是连续的，由关节控制器插件驱动。
惯性测量单元安装在底盘上。仿真插件配置为以高频率发布线性加速度和角速度数据。这对于未来的传感器融合任务至关重要。
地面真值发布器提供准确的位置数据，用于验证状态估计算法。

C. 控制系统实现
利用操作系统的模块化特性，软件控制栈已实现并经过测试。
系统启动配置协调仿真、机器人状态发布器、通信桥接和可视化工具的初始化。这确保了开发环境的工作流简化。它使用包含机制来封装标准仿真启动文件，传递特定参数以加载自定义世界。
通信桥接已配置为将仿真主题映射到控制系统主题。具体而言，它桥接各个车轮的速度指令信号和传感器数据流。这个桥接至关重要，因为它将控制系统使用的数据分发服务消息转换为模拟器使用的传输层。
使用客户端库开发了一个自定义手动控制节点。与标准遥操作工具不同，该节点将输入信号直接映射到差速车轮速度。
该逻辑为每个车轮控制器主题初始化发布器。它以原始模式捕获输入信号以确保立即响应。
它实现了一个运动学模型，其中输入增加或减少所有车轮的线速度目标，并在左右车轮对之间引入差速以引起旋转。这在微控制器上实现之前验证了差速驱动混合逻辑。
该节点包括一个安全功能，可立即将所有速度归零，这是在受限管道环境中进行测试的关键功能。
机器人成功在生成的管道世界中进行手动控制移动。差速转向机制按预期工作，允许机器人原地转向并在环境中机动。

D. 可视化与调试
可视化工具已集成到工作流中。配置设置为可视化机器人模型、坐标系和传感器数据。这允许实时调试运动链和传感器放置。我可以直观地验证车轮变换随着机器人的移动而正确更新，确认关节状态发布器功能正常。

E. 过程化环境生成细节
过程化生成模块代表了一项重要的软件资产。它解决了在无需手动建模的情况下创建多样化测试环境的挑战。
该算法使用直线和圆弧序列定义路径。它将这些段离散化为点，并计算每个点的切向量。
对于路径上的每个点，它生成垂直于切线的顶点圆。然后将这些顶点三角化以形成管道表面。
该模块自动处理最后一点和第一点之间的连接，以确保网格的水密性，这对于物理引擎正确检测碰撞至关重要。

总之，控制系统的基础工作已经完成。仿真环境运行正常，机器人模型物理上准确，基本控制回路已闭合。这为实现高级自主功能奠定了基础。

## 存在的问题及拟解决措施

尽管取得了进展，但在仿真和初步测试阶段仍发现了一些挑战。

1. 曲面上的控制挑战
目前的差速驱动模型假设表面平坦。在圆柱形管道中，随着机器人沿管壁移动，车轮的接触点会发生变化。这改变了有效轮距并导致不可预测的打滑，使得精确的开环控制变得困难。
我将实现一个闭环控制系统。通过使用惯性传感器数据，特别是陀螺仪，我可以实现一个反馈控制器来修正车轮速度以保持稳定性。此外，我将研究曲面上滑移转向车辆的运动学建模，以优化控制输入。

2. 传感器信号质量
虽然仿真目前提供地面真值，但模拟的惯性传感器包含噪声。初步测试表明，积分原始加速度计数据会导致状态在几秒钟内发生显著漂移。
将使用定位包实现状态估计滤波器。这将融合惯性数据与从关节状态计算出的车轮里程计。在未来的硬件实现中，我还将集成额外的感知传感器以提供绝对修正。

3. 系统延迟
在仿真中，输入与机器人响应之间存在明显的延迟。这是由于通信桥接的开销和解释型语言的执行速度造成的。
对于仿真，我将优化服务质量设置以减少延迟。对于最终的嵌入式系统，这个问题将通过在微控制器上以编译语言直接运行控制回路来缓解，从而消除操作系统和桥接的开销。

4. 感知局限性
机器人目前在没有环境感知的情况下运行。它无法自主检测管道几何形状或障碍物。
下一阶段涉及向机器人模型添加模拟测距传感器。然后，我将开发感知算法，利用距离数据来理解机器人相对于管壁的位置。

## 下一步工作计划

该项目下一阶段的计划分为三个主要轨道：高级控制、嵌入式软件开发和硬件集成。

1. 高级控制策略（第1-2个月）
当前的重点是在仿真中从手动控制过渡到自动稳定性控制。
我将实现基于反馈的稳定性算法。利用模拟测距传感器，机器人将调整其转向以在管道内保持稳定的姿态。
我将开发一个运动规划模块，识别最佳路径并生成速度指令以跟随该路径，确保平滑的运动曲线。
我将调整机器人描述和世界模型中的摩擦系数，以更好地匹配真实下水道的预期条件，如潮湿的混凝土或塑料。

2. 嵌入式系统迁移（第3-4个月）
一旦控制逻辑在仿真中得到验证，我将开始将核心算法移植到嵌入式平台。这是满足项目实时要求的关键步骤。
我将在微控制器上设置实时操作系统。这对应于开发多任务调度程序的项目目标。任务将分为：
传感器采集任务：高优先级任务，通过直接内存访问读取惯性和编码器数据，以最小化处理器负载。
控制回路任务：周期性任务，执行反馈或运动学混合算法。
通信任务：较低优先级的任务，处理总线消息和调试接口。
我将实现电机驱动器的底层脉宽调制生成。这涉及配置微控制器的高级控制定时器，以生成带有死区插入的互补信号，防止硬件损坏。
我将实现控制器局域网通信栈。这将允许主控制器可靠地与电机驱动器和传感器模块通信。我还将实现一个串行桥接，允许嵌入式系统与高级计算机对话，使机器人能够直接从微控制器接收指令。

3. 传感器融合与状态估计（第4-5个月）
我将为微控制器上的真实惯性传感器开发底层驱动程序。我将实现硬件抽象层，将传感器逻辑与特定硬件引脚解耦。
我将实现项目目标中提到的传感器数据同步接口。这确保惯性数据和车轮编码器数据在被融合算法处理之前已打上时间戳并对齐。我将使用硬件定时器作为公共时基。
我将尝试在微控制器上运行简化的状态估计滤波器，以实时估计机器人的方向，这对于管道中的稳定性至关重要。这将涉及针对处理器架构优化数学运算。

4. 硬件在环测试（第6个月）
在最终机器人组装之前，我将进行硬件在环测试。微控制器板将连接到仿真计算机。仿真将向微控制器发送虚拟传感器数据，微控制器将向仿真发送电机指令。这在不冒物理机器人风险的情况下验证了嵌入式代码。

## 对毕业设计（论文）能否按期完成情况的评价
基于目前的进展，项目正按计划进行，有望在2026年5月的截止日期前完成。仿真环境的成功建立是降低重大技术风险的一个重要里程碑。通过首先在软件中验证运动学模型和控制逻辑，我减少了硬件调试所需的时间。

剩余的任务定义明确，并遵循从高级软件到底层固件的逻辑进程。并行开发策略——在设计嵌入式架构的同时在仿真中完善算法——确保了一个领域的瓶颈不会拖延整个项目。团队有信心最终原型将满足所有规定的要求，包括多任务调度和精确的电机控制，从而为下水道检测提供高质量的工程解决方案。
