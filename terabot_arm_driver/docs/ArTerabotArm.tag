<?xml version='1.0' encoding='ISO-8859-1' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>ArTerabotArm.cpp</name>
    <path>/home/reed/Aria/ArTerabotArm/src/</path>
    <filename>ArTerabotArm_8cpp</filename>
    <includes id="ArTerabotArm_8h" name="ArTerabotArm.h" local="yes" imported="no">ArTerabotArm.h</includes>
    <member kind="define">
      <type>#define</type>
      <name>DEBUG</name>
      <anchorfile>ArTerabotArm_8cpp.html</anchorfile>
      <anchor>a3dfa58b1c5c2943dd49d8aa1981d377d</anchor>
      <arglist>(x)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>ArTerabotArm.h</name>
    <path>/home/reed/Aria/ArTerabotArm/include/</path>
    <filename>ArTerabotArm_8h</filename>
    <class kind="class">ArTerabotArm</class>
    <class kind="class">ArTerabotArm::ByteCommand</class>
    <class kind="class">ArTerabotArm::Command</class>
    <class kind="class">ArTerabotArm::DataMessage</class>
    <class kind="class">ArTerabotArm::IntCommand</class>
    <class kind="class">ArTerabotArm::JogCommand</class>
    <class kind="class">ArTerabotArm::LimitsCommand</class>
    <class kind="class">ArTerabotArm::MaxVelCommand</class>
    <class kind="class">ArTerabotArm::Message</class>
    <class kind="class">ArTerabotArm::StatusMessage</class>
    <member kind="define">
      <type>#define</type>
      <name>_AR_NAN</name>
      <anchorfile>ArTerabotArm_8h.html</anchorfile>
      <anchor>a0af11300240e05c93db74c2c3459bdda</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>mogsAndArmServer.cpp</name>
    <path>/home/reed/Aria/ArTerabotArm/examples/</path>
    <filename>mogsAndArmServer_8cpp</filename>
    <includes id="ArTerabotArm_8h" name="ArTerabotArm.h" local="yes" imported="no">ArTerabotArm.h</includes>
    <member kind="function">
      <type>const char *</type>
      <name>getGyroStatusString</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>af85900f5c0a1a44fd3573732f9cdf8ef</anchor>
      <arglist>(ArRobot *robot)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>goalDone</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>a1ea7fe17bd8aa565fb04e5e9a0c668b7</anchor>
      <arglist>(ArPose goalPos)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>goalFailed</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>a26bd158bbefd0a0567ed7466148604ae</anchor>
      <arglist>(ArPose goalPos)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>locFailed</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>a0de4b18dc08d4060082bfcac2924c5a1</anchor>
      <arglist>(int n)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>logOptions</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>ad3b42c34a9ad6ed513a76481b280254c</anchor>
      <arglist>(const char *progname)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>pathPlanStateChanged</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>ac51ea0964154e2a26302808cc01d8ca7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>gyroErrored</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>a2e672fb29f4d5a6b5266bd53bf180a52</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>ArPathPlanningInterface *</type>
      <name>pathPlanningTask</name>
      <anchorfile>mogsAndArmServer_8cpp.html</anchorfile>
      <anchor>a43c1e2b5f20daa310a59e519dbb15388</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>terabotArmDemo.cpp</name>
    <path>/home/reed/Aria/ArTerabotArm/examples/</path>
    <filename>terabotArmDemo_8cpp</filename>
    <includes id="ArTerabotArm_8h" name="ArTerabotArm.h" local="yes" imported="no">ArTerabotArm.h</includes>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>defaultJointSpeed</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>ad2b4caffe7163769dffde5e5153a45a3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>forwardReady</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>a1c4fca2bf8133bda3b503454ea6cfd05</anchor>
      <arglist>[5]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>left</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>a6515953aa383c5c95b44477a4ac73853</anchor>
      <arglist>[5]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>park</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>ad56596de36ab69cd46a089ae716a6a21</anchor>
      <arglist>[5]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>pickUpFloor</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>a14a3a9755826d5c7b173521f8870531c</anchor>
      <arglist>[5]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>right</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>a8060c7ce7aabf0e6b0afc194b5af09f8</anchor>
      <arglist>[5]</arglist>
    </member>
    <member kind="variable" static="yes">
      <type>static float</type>
      <name>straightForward</name>
      <anchorfile>terabotArmDemo_8cpp.html</anchorfile>
      <anchor>a516e09205964e1387ad34f887130eef6</anchor>
      <arglist>[5]</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ArTerabotArm</name>
    <filename>classArTerabotArm.html</filename>
    <class kind="class">ArTerabotArm::ByteCommand</class>
    <class kind="class">ArTerabotArm::Command</class>
    <class kind="class">ArTerabotArm::DataMessage</class>
    <class kind="class">ArTerabotArm::IntCommand</class>
    <class kind="class">ArTerabotArm::JogCommand</class>
    <class kind="class">ArTerabotArm::LimitsCommand</class>
    <class kind="class">ArTerabotArm::MaxVelCommand</class>
    <class kind="class">ArTerabotArm::Message</class>
    <class kind="class">ArTerabotArm::StatusMessage</class>
    <member kind="enumeration">
      <name>JointStatus</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>SERVO_ACTIVE</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900ca2eae15530a73072ca6e7106f096f5f7f</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>ON_TARGET</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900cab4b4039b89c41327fe2bb6a4e41a4c1c</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>IN_PROGRESS</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900cafb81283d3f95825272af076facf849d4</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>CONTROLLER_OVER_TEMP</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900cadc1072ff11f59907bfc8f42e4cfa722d</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>FOLLOWING_ERROR</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900ca9ca275cc19b1e33b51fe7a71eed18694</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue">
      <name>RESET_COMPLETE</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2109e6334a3f1bd5c33048e435d8900ca0e58c82e13d170d7e277fbe9488bb3af</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>ArTerabotArm</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a523e60254c568a3e1ff15cba9638c5b0</anchor>
      <arglist>(ArRobot *robot=NULL, const char *port=NULL)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>closeGripper</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a3abb54a56638eb9b4b5b0498768c67be</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disable</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a0e42240ec3d07ff4e35e9c7daf25df7e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>disableArm</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a4ecf818a48199b4e621d33b1763cb4af</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>enable</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>aa5f1c5b66c7544fd519f35bc80df477c</anchor>
      <arglist>(unsigned char state=0x3F)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>enableArm</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a5a28d0c8c28e747b85bdc9844d6fa747</anchor>
      <arglist>(unsigned char state=0x3F)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getArmPos</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a61699a5de8263a3627cb6e37342331c6</anchor>
      <arglist>(float *pos1, float *pos2, float *pos3, float *pos4, float *pos5)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getArmPos</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a04e562484b3f5e89b7283a1560c732cb</anchor>
      <arglist>(float pos[5])</arglist>
    </member>
    <member kind="function">
      <type>float *</type>
      <name>getArmPos</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a2eafd9a6afab0cf05b9397195f105cce</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>ArDeviceConnection *</type>
      <name>getDeviceConnection</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ae3d48eda191a2f03350a458f707601c9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>unsigned short</type>
      <name>getGripperStatus</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a6067d4dd74a355f46ac975b8fa75bf87</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>float</type>
      <name>getJointPos</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a35af400788d5773bbde3938f937bf475</anchor>
      <arglist>(int joint)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getJointStatus</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ab657f76c8e4f81059c05c2d94e736804</anchor>
      <arglist>(unsigned short *s1, unsigned short *s2, unsigned short *s3, unsigned short *s4, unsigned short *s5)</arglist>
    </member>
    <member kind="function">
      <type>unsigned short *</type>
      <name>getJointStatus</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a6fb6782586f95bac22ba1f9d4a173374</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>grip</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a91dae102a2104010c61b3f56cccce5ff</anchor>
      <arglist>(short int value)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>halt</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>af5ab87d16dac0cb8d98e748f0472c16b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>isOpen</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ace60138a4d2ccb3f13211c98d5237b2e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>moveArm</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>afc2041fe206bc799b64879af35952e5d</anchor>
      <arglist>(float pos1=_AR_NAN, float pos2=_AR_NAN, float pos3=_AR_NAN, float pos4=_AR_NAN, float pos5=_AR_NAN)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>moveArm</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>adc667689641c3be96292a9ff3d4bc26f</anchor>
      <arglist>(float pose[5])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>moveJoint</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a4206bcc30d5902d02eb9c6fe6a0dd5ba</anchor>
      <arglist>(int joint, float pos)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>moving</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>afcad054231d495424507d5d00c363468</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>open</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ad984b38432954009652c76238ecedcf7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>openGripper</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ae5e95215e145d780787bb0ce3c93189a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>powerOff</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>af30b8eac2e7ea64698ccbbf0f98fe2d9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>powerOn</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a8d5d76e655b20f7cb0f4a33246817e1c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>read</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a63a67084fa4c722f0e333f496b3e5546</anchor>
      <arglist>(int timeout=10)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>requestStatus</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ae52888deca8dd383548bc21d131faefc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>reset</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>aaf697ad480669927c0eb4986958dd722</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setAllJointSpeeds</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a5572cbb3bc4afca8242c1143f554afc2</anchor>
      <arglist>(float vel)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setDeviceConnection</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a0c9afcd41429edc826aa8a343ba50af6</anchor>
      <arglist>(ArDeviceConnection *conn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setInputDeviceConnection</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a5f3e914981e222eccd34321f545e035d</anchor>
      <arglist>(ArDeviceConnection *conn)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setJointLimits</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a5f4daab24ed960b72c04ad1192aef088</anchor>
      <arglist>(float min[5], float max[5])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setJointSpeed</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ad77aa43103e87b05cd94373c500630ae</anchor>
      <arglist>(int joint, float vel)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMaxSpeeds</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a3186d6dfd60725c90a659521abd1d6ef</anchor>
      <arglist>(float max[5])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setSpeeds</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a68675d68bf5b74752d631e08a1edbd21</anchor>
      <arglist>(float speed[5])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopGripper</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>ad2441db9bbe6c07ad98cfe911079aa9a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ArTerabotArm</name>
      <anchorfile>classArTerabotArm.html</anchorfile>
      <anchor>a496a487175c1820540498de53ec42e39</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
</tagfile>
