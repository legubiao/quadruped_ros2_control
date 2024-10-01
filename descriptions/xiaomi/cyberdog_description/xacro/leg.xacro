<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find cyberdog_description)/xacro/transmission.xacro"/>

  <xacro:macro name="leg" params="name mirror mirror_dae front_hind front_hind_dae *origin">

    <joint name="${name}_hip_joint" type="revolute">
      <xacro:insert_block name="origin"/>
      <parent link="body"/>
      <child link="${name}_abad"/>
      <axis xyz="1 0 0"/>
      <dynamics damping="${abad_damping}" friction="${abad_friction}"/>
      <xacro:if value="${(mirror_dae == True)}">
        <limit effort="${abad_motorTauMax*abadGearRatio}"
               velocity="${abad_motorVelMax}" lower="${abad_min*PI/180.0}"
               upper="${abad_max*PI/180.0}"/>
      </xacro:if>
      <xacro:if value="${(mirror_dae == False)}">
        <limit effort="${abad_motorTauMax*abadGearRatio}"
               velocity="${abad_motorVelMax}" lower="${-abad_max*PI/180.0}"
               upper="${-abad_min*PI/180.0}"/>
      </xacro:if>
    </joint>

    <link name="${name}_abad">
      <visual>
        <xacro:if value="${(mirror_dae == True) and (front_hind_dae == True)}">
          <origin rpy="0 0 0" xyz="0 0 0"/>
        </xacro:if>
        <xacro:if value="${(mirror_dae == False) and (front_hind_dae == True)}">
          <origin rpy="${PI} 0 0" xyz="0 0 0"/>
        </xacro:if>
        <xacro:if value="${(mirror_dae == True) and (front_hind_dae == False)}">
          <origin rpy="0 ${PI} 0" xyz="0 0 0"/>
        </xacro:if>
        <xacro:if value="${(mirror_dae == False) and (front_hind_dae == False)}">
          <origin rpy="${PI} ${PI} 0" xyz="0 0 0"/>
        </xacro:if>
        <geometry>
          <mesh filename="file://$(find cyberdog_description)/meshes/abad.dae" scale="1 1 1"/>
        </geometry>
      </visual>
      <collision>
        <origin rpy="${PI/2.0} 0 0" xyz="0 0 0"/>
        <geometry>
          <cylinder length="${abad_length}" radius="${abad_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <origin rpy="0 0 0"
                xyz="${abad_com_x*front_hind} ${abad_com_y*mirror} ${abad_com_z}"/>
        <mass value="${abad_mass}"/>
        <inertia ixx="${abad_ixx}" ixy="${abad_ixy*mirror*front_hind}"
                 ixz="${abad_ixz*front_hind}" iyy="${abad_iyy}" iyz="${abad_iyz*mirror}"
                 izz="${abad_izz}"/>
      </inertial>
    </link>

    <joint name="${name}_abad_rotor_fix" type="fixed">
      <origin rpy="0 0 0" xyz="${-abad_rotor_offset*front_hind} 0 0"/>
      <parent link="${name}_abad"/>
      <child link="${name}_abad_rotor"/>
    </joint>

    <!-- this link is only for abad rotor inertial -->
    <link name="${name}_abad_rotor">
      <inertial>
        <origin rpy="0 0 0" xyz="${rotor_com_x} ${rotor_com_y} ${rotor_com_z}"/>
        <mass value="${rotor_mass}"/>
        <inertia ixx="${rotor_ixx}" ixy="${rotor_ixy}" ixz="${rotor_ixz}" iyy="${rotor_iyy}" iyz="${rotor_iyz}"
                 izz="${rotor_izz}"/>
      </inertial>
    </link>

    <joint name="${name}_abad_fixed" type="fixed">
      <origin rpy="0 0 0" xyz="0 ${(hip_offset-hip_shoulder_length)*mirror} 0"/>
      <parent link="${name}_abad"/>
      <child link="${name}_hip_shoulder"/>
    </joint>

    <link name="${name}_hip_shoulder">
      <collision>
        <origin rpy="${PI/2.0} 0 0" xyz="0 0 0"/>
        <geometry>
          <cylinder length="${hip_shoulder_length}" radius="${hip_shoulder_radius}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${name}_thigh_joint" type="revolute">
      <origin rpy="0 0 0" xyz="0 ${hip_offset*mirror} 0"/>
      <parent link="${name}_abad"/>
      <child link="${name}_hip"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="${hip_damping}" friction="${hip_friction}"/>
      <xacro:if value="${front_hind_dae == True}">
        <limit effort="${hip_motorTauMax*hipGearRatio}"
               velocity="${hip_motorVelMax}" lower="${hip_f_min*PI/180.0}"
               upper="${hip_f_max*PI/180.0}"/>
      </xacro:if>
      <xacro:if value="${front_hind_dae == False}">
        <limit effort="${hip_motorTauMax*hipGearRatio}"
               velocity="${hip_motorVelMax}" lower="${hip_h_min*PI/180.0}"
               upper="${hip_h_max*PI/180.0}"/>
      </xacro:if>
    </joint>

    <link name="${name}_hip">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <xacro:if value="${mirror_dae == True}">
            <mesh filename="file://$(find cyberdog_description)/meshes/hip.dae"
                  scale="1 1 1"/>
          </xacro:if>
          <xacro:if value="${mirror_dae == False}">
            <mesh filename="file://$(find cyberdog_description)/meshes/hip_mirror.dae"
                  scale="1 1 1"/>
          </xacro:if>
        </geometry>
      </visual>
      <collision>
        <origin rpy="0 ${PI/2.0} 0" xyz="0 0 ${-hip_length/2.0}"/>
        <geometry>
          <box size="${hip_length} ${hip_width} ${hip_height}"/>
        </geometry>
      </collision>
      <inertial>
        <origin rpy="0 0 0" xyz="${hip_com_x} ${hip_com_y*mirror} ${hip_com_z}"/>
        <mass value="${hip_mass}"/>
        <inertia ixx="${hip_ixx}" ixy="${hip_ixy*mirror}" ixz="${hip_ixz}" iyy="${hip_iyy}"
                 iyz="${hip_iyz*mirror}" izz="${hip_izz}"/>
      </inertial>
    </link>

    <joint name="${name}_hip_rotor_fix" type="fixed">
      <origin rpy="0 0 0" xyz="0 ${hip_rotor_offset*mirror} 0"/>
      <parent link="${name}_hip"/>
      <child link="${name}_hip_rotor"/>
    </joint>

    <!-- this link is only for hip rotor inertial -->
    <link name="${name}_hip_rotor">
      <inertial>
        <origin rpy="0 0 0" xyz="${rotor_com_x} ${rotor_com_y} ${rotor_com_z}"/>
        <mass value="${rotor_mass}"/>
        <inertia ixx="${rotor_ixx}" ixy="${rotor_ixy}" ixz="${rotor_ixz}" iyy="${rotor_iyy}" iyz="${rotor_iyz}"
                 izz="${rotor_izz}"/>
      </inertial>
    </link>

    <joint name="${name}_knee_rotor_fix" type="fixed">
      <origin rpy="0 0 0" xyz="0 ${knee_rotor_offset*mirror} 0"/>
      <parent link="${name}_hip"/>
      <child link="${name}_knee_rotor"/>
    </joint>

    <!-- this link is only for knee rotor inertial -->
    <link name="${name}_knee_rotor">
      <inertial>
        <origin rpy="0 0 0" xyz="${rotor_com_x} ${rotor_com_y} ${rotor_com_z}"/>
        <mass value="${rotor_mass}"/>
        <inertia ixx="${rotor_ixx}" ixy="${rotor_ixy}" ixz="${rotor_ixz}" iyy="${rotor_iyy}" iyz="${rotor_iyz}"
                 izz="${rotor_izz}"/>
      </inertial>
    </link>

    <joint name="${name}_calf_joint" type="revolute">
      <origin rpy="0 0 0" xyz="0 0 ${-hip_length}"/>
      <parent link="${name}_hip"/>
      <child link="${name}_knee"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="${hip_damping}" friction="${hip_friction}"/>
      <limit effort="${knee_motorTauMax*kneeGearRatio}"
             velocity="${knee_motorVelMax}" lower="${knee_min*PI/180.0}"
             upper="${knee_max*PI/180.0}"/>
    </joint>

    <link name="${name}_knee">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="file://$(find cyberdog_description)/meshes/knee.dae" scale="1 1 1"/>
        </geometry>
      </visual>
      <collision>
        <origin rpy="0 ${PI} 0" xyz="0 0 ${-knee_length/2.0}"/>
        <geometry>
          <box size="${knee_height} ${knee_width} ${knee_length}"/>
        </geometry>
      </collision>
      <collision name="${name}_knee_rubber">
        <origin rpy="0 ${PI} 0" xyz="${-knee_rubber/2.0} 0 -0.007"/>
        <geometry>
          <sphere radius="0.016"/>
        </geometry>
        <surface>
          <contact>
            <ode>
              <max_vel>0.00001</max_vel>
              <min_depth>0.0</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <inertial>
        <origin rpy="0 0 0" xyz="${knee_com_x} ${knee_com_y} ${knee_com_z}"/>
        <mass value="${knee_mass}"/>
        <inertia ixx="${knee_ixx}" ixy="${knee_ixy}" ixz="${knee_ixz}" iyy="${knee_iyy}"
                 iyz="${knee_iyz}" izz="${knee_izz}"/>
      </inertial>
    </link>

    <joint name="${name}_foot_fixed" type="fixed">
      <origin rpy="0 0 0" xyz="0.0055 0 ${-knee_length+foot_radius/2.0}"/>
      <parent link="${name}_knee"/>
      <child link="${name}_foot"/>
    </joint>

    <link name="${name}_foot">
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <sphere radius="${foot_radius-0.01}"/>
        </geometry>
      </visual>
      <collision>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <sphere radius="${foot_radius}"/>
        </geometry>
        <surface>
          <contact>
            <ode>
              <max_vel>0.001</max_vel>
              <min_depth>0.0</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <xacro:leg_transmission name="${name}"/>

    <gazebo reference="${name}_abad">
      <mu1>0.2</mu1>
      <mu2>0.2</mu2>
      <visual>
        <material>
          <ambient>.5 .5 .5 1.0</ambient>
          <diffuse>.5 .5 .5 1.0</diffuse>
          <specular>.5 .5 .5 1.0</specular>
        </material>
      </visual>
    </gazebo>
    <gazebo reference="${name}_hip">
      <mu1>0.2</mu1>
      <mu2>0.2</mu2>
      <self_collide>1</self_collide>
      <visual>
        <material>
          <ambient>.05 .05 .05 1.0</ambient>
          <diffuse>.05 .05 .05 1.0</diffuse>
          <specular>.05 .05 .05 1.0</specular>
        </material>
      </visual>
    </gazebo>
    <gazebo reference="${name}_knee">
      <mu1>0.7</mu1>
      <mu2>0.7</mu2>
      <self_collide>1</self_collide>
      <visual>
        <material>
          <ambient>.5 .5 .5 1.0</ambient>
          <diffuse>.5 .5 .5 1.0</diffuse>
          <specular>.5 .5 .5 1.0</specular>
        </material>
      </visual>
    </gazebo>
    <gazebo reference="${name}_foot">
      <mu1>0.7</mu1>
      <mu2>0.7</mu2>
      <self_collide>1</self_collide>
      <visual>
        <material>
          <ambient>.5 .5 .5 1.0</ambient>
          <diffuse>.5 .5 .5 1.0</diffuse>
          <specular>.5 .5 .5 1.0</specular>
        </material>
      </visual>
    </gazebo>

  </xacro:macro>
</robot>