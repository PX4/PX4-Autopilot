# 机架参考

:::info
**This list is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/px4airframes/markdownout.py) from the source code** using the build command: `make airframe_metadata`.
:::

This page lists all supported airframes and types including the motor assignment and numbering.
The motors in **green** rotate clockwise, the ones in **blue** counterclockwise.

**AUX** channels may not be present on some flight controllers.
If present, PWM AUX channels are commonly labelled **AUX OUT**.

<style>
div.frame_common table, div.frame_common table {
   display: table;
   table-layout: fixed;
   margin-bottom: 5px;
}

div.frame_common table {
   float: right;
   width: 70%;
}

div.frame_common img {
  max-height: 180px;
  width: 29%;
  padding-top: 10px;
}

div.frame_variant table {
   width: 100%;
}

div.frame_variant th:nth-child(1) {
  width: 30%;
  }

div.frame_variant tr > * {
    vertical-align : top;
}

div.frame_variant td, div.frame_variant th {
  text-align : left;
}
</style>

## 2D Space Robot

### Space Robot

<div class="frame_common">
<img src="../../assets/airframes/types/AirframeUnknown.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="2d_space_robot_space_robot_kth_space_robot">
 <td>KTH Space Robot</td>
 <td>Maintainer: DISCOWER<p><code>SYS_AUTOSTART</code> = 70000</p></td>
</tr>
</tbody>
</table>
</div>

## 旋翼机

### 旋翼机

<div class="frame_common">
<img src="../../assets/airframes/types/Airship.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 右舷推进器</li><li><b>电机2</b>: 左舷推进器</li><li><b>电机3</b>: 尾部推进器</li><li><b>舵机1</b>: 推力倾斜</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="airship_airship_cloudship">
 <td>Cloudship</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 2507</p></td>
</tr>
</tbody>
</table>
</div>

## 飞机

### 飞机

<div class="frame_common">
<img src="../../assets/airframes/types/Autogyro.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 油门</li><li><b>Servo1</b>: rotor_head_L</li><li><b>Servo2</b>: rotor_head_R</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="autogyro_autogyro_thunderfly_auto-g2">
 <td><a href="https://github.com/ThunderFly-aerospace/Auto-G2/">ThunderFly Auto-G2</a></td>
 <td>Maintainer: ThunderFly s.r.o., Roman Dvorak &lt;dvorakroman@thunderfly.cz&gt;<p><code>SYS_AUTOSTART</code> = 17002</p><br><b>Specific Outputs:</b><ul><li><b>Servo3</b>: elevator</li><li><b>Servo4</b>: rudder</li><li><b>Servo5</b>: rudder (second, optional)</li><li><b>Servo6</b>: wheel</li></ul></td>
</tr>
<tr id="autogyro_autogyro_thunderfly_tf-g2">
 <td><a href="https://github.com/ThunderFly-aerospace/TF-G2/">ThunderFly TF-G2</a></td>
 <td>Maintainer: ThunderFly s.r.o., Roman Dvorak &lt;dvorakroman@thunderfly.cz&gt;<p><code>SYS_AUTOSTART</code> = 17003</p><br><b>Specific Outputs:</b><ul><li><b>Servo3</b>: rudder</li></ul></td>
</tr>
</tbody>
</table>
</div>

## Balloon

### Balloon

<div class="frame_common">
<img src="../../assets/airframes/types/Balloon.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="balloon_balloon_thunderfly_balloon_tf-b1">
 <td><a href="https://github.com/ThunderFly-aerospace/TF-B1/">ThunderFly balloon TF-B1</a></td>
 <td>Maintainer: ThunderFly s.r.o.<p><code>SYS_AUTOSTART</code> = 18001</p></td>
</tr>
</tbody>
</table>
</div>

## 旋翼机

### Dodecarotor cox

<div class="frame_common">
<img src="../../assets/airframes/types/DodecaRotorXCoaxial.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_dodecarotor_cox_generic_dodecarotor_cox_geometry">
 <td>Generic Dodecarotor cox geometry</td>
 <td>Maintainer: William Peale &lt;develop707@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 24001</p></td>
</tr>
</tbody>
</table>
</div>

### Helicopter

<div class="frame_common">
<img src="../../assets/airframes/types/Helicopter.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_helicopter_generic_helicopter_(tail_esc)">
 <td>通用直升机(Tail ESC)</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 16001</p></td>
</tr>
</tbody>
</table>
</div>

### Hexarotor +

<div class="frame_common">
<img src="../../assets/airframes/types/HexaRotorPlus.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_hexarotor_+_generic_hexarotor_+_geometry">
 <td>Generic Hexarotor + geometry</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 7001</p></td>
</tr>
</tbody>
</table>
</div>

### Hexarotor Coaxial

<div class="frame_common">
<img src="../../assets/airframes/types/Y6B.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 右前顶部; 角度:60; 方向:顺时针</li><li><b>电机2</b>: 右前底部; 角度:60; 方向:逆时针</li><li><b>电机3</b>: 后顶部; 角度:180; 方向:顺时针</li><li><b>电机4</b>: 后底部; 角度:180; 方向:逆时针</li><li><b>电机5</b>: 左前顶部; 角度:-60; 方向:顺时针</li><li><b>电机6</b>: 左前底部; 角度:-60; 方向:逆时针</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_hexarotor_coaxial_generic_hexarotor_coaxial_geometry">
 <td>Generic Hexarotor coaxial geometry</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 11001</p></td>
</tr>
</tbody>
</table>
</div>

### Hexarotor x

<div class="frame_common">
<img src="../../assets/airframes/types/HexaRotorX.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_hexarotor_x_generic_hexarotor_x_geometry">
 <td>Generic Hexarotor x geometry</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 6001</p></td>
</tr>
<tr id="copter_hexarotor_x_uvify_draco-r">
 <td>UVify Draco-R</td>
 <td>Maintainer: Hyon Lim &lt;lim@uvify.com&gt;<p><code>SYS_AUTOSTART</code> = 6002</p></td>
</tr>
</tbody>
</table>
</div>

### Octorotor +

<div class="frame_common">
<img src="../../assets/airframes/types/OctoRotorPlus.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_octorotor_+_generic_octocopter_+_geometry">
 <td>Generic Octocopter + geometry</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 9001</p></td>
</tr>
</tbody>
</table>
</div>

### Octorotor Coaxial

<div class="frame_common">
<img src="../../assets/airframes/types/OctoRotorXCoaxial.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 电机1</li><li><b>电机2</b>: 电机2</li><li><b>电机3</b>: 电机3</li><li><b>电机4</b>: 电机4</li><li><b>电机5</b>: 电机5</li><li><b>电机6</b>: 电机6</li><li><b>电机7</b>: 电机7</li><li><b>电机8</b>: 电机8</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_octorotor_coaxial_generic_10__octo_coaxial_geometry">
 <td>Generic 10" Octo coaxial geometry</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 12001</p></td>
</tr>
</tbody>
</table>
</div>

### Octorotor x

<div class="frame_common">
<img src="../../assets/airframes/types/OctoRotorX.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_octorotor_x_generic_octocopter_x_geometry">
 <td>Generic Octocopter X geometry</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 8001</p></td>
</tr>
</tbody>
</table>
</div>

### Quadrotor +

<div class="frame_common">
<img src="../../assets/airframes/types/QuadRotorPlus.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_quadrotor_+_generic_quad_+_geometry">
 <td>Generic Quad + geometry</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 5001</p></td>
</tr>
</tbody>
</table>
</div>

### Quadrotor H

<div class="frame_common">
<img src="../../assets/airframes/types/QuadRotorH.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_quadrotor_h_betafpv_beta75x_2s_brushless_whoop">
 <td>BetaFPV Beta75X 2S Brushless Whoop</td>
 <td>Maintainer: Beat Kueng &lt;beat-kueng@gmx.net&gt;<p><code>SYS_AUTOSTART</code> = 4041</p></td>
</tr>
</tbody>
</table>
</div>

### Quadrotor x

<div class="frame_common">
<img src="../../assets/airframes/types/QuadRotorX.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_quadrotor_x_generic_quadcopter">
 <td>Generic Quadcopter</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 4001</p></td>
</tr>
<tr id="copter_quadrotor_x_s500_generic">
 <td>S500 Generic</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 4014</p></td>
</tr>
<tr id="copter_quadrotor_x_holybro_s500">
 <td>Holybro S500</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 4015</p></td>
</tr>
<tr id="copter_quadrotor_x_px4_vision_dev_kit_v1">
 <td>PX4 Vision Dev Kit v1</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 4016</p></td>
</tr>
<tr id="copter_quadrotor_x_nxp_hovergames">
 <td>NXP HoverGames</td>
 <td>Maintainer: Iain Galloway &lt;iain.galloway@nxp.com&gt;<p><code>SYS_AUTOSTART</code> = 4017</p></td>
</tr>
<tr id="copter_quadrotor_x_holybro_x500_v2">
 <td>Holybro X500 V2</td>
 <td>Maintainer: Farhang Naderi &lt;farhang.nba@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 4019</p></td>
</tr>
<tr id="copter_quadrotor_x_px4_vision_dev_kit_v1.5">
 <td>PX4 Vision Dev Kit v1.5</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 4020</p></td>
</tr>
<tr id="copter_quadrotor_x_generic_250_racer">
 <td>Generic 250 Racer</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 4050</p></td>
</tr>
<tr id="copter_quadrotor_x_holybro_qav250">
 <td><a href="https://docs.px4.io/main/en/frames_multicopter/holybro_qav250_pixhawk4_mini.html">HolyBro QAV250</a></td>
 <td>Maintainer: Beat Kueng &lt;beat-kueng@gmx.net&gt;<p><code>SYS_AUTOSTART</code> = 4052</p></td>
</tr>
<tr id="copter_quadrotor_x_holybro_kopis_2">
 <td>Holybro Kopis 2</td>
 <td>Maintainer: Beat Kueng &lt;beat@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 4053</p></td>
</tr>
<tr id="copter_quadrotor_x_advanced_technology_labs_(atl)_mantis_edu">
 <td>Advanced Technology Labs (ATL) Mantis EDU</td>
 <td><p><code>SYS_AUTOSTART</code> = 4061</p></td>
</tr>
<tr id="copter_quadrotor_x_uvify_ifo">
 <td>UVify IFO</td>
 <td>Maintainer: Hyon Lim &lt;lim@uvify.com&gt;<p><code>SYS_AUTOSTART</code> = 4071</p></td>
</tr>
<tr id="copter_quadrotor_x_uvify_ifo">
 <td>UVify IFO</td>
 <td>Maintainer: Hyon Lim &lt;lim@uvify.com&gt;<p><code>SYS_AUTOSTART</code> = 4073</p></td>
</tr>
<tr id="copter_quadrotor_x_coex_clover_4">
 <td>COEX Clover 4</td>
 <td>Maintainer: Oleg Kalachev &lt;okalachev@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 4500</p></td>
</tr>
<tr id="copter_quadrotor_x_droneblocks_dexi_5">
 <td>Droneblocks DEXI 5</td>
 <td>Maintainer: Alex klimaj &lt;alex@arkelectron.com&gt;<p><code>SYS_AUTOSTART</code> = 4601</p></td>
</tr>
<tr id="copter_quadrotor_x_crazyflie_2.1">
 <td>Crazyflie 2.1</td>
 <td>Maintainer: Dennis Shtatov &lt;densht@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 4901</p></td>
</tr>
</tbody>
</table>
</div>

### 仿真

<div class="frame_common">
<img src="../../assets/airframes/types/AirframeSimulation.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_simulation_hil_quadcopter_x">
 <td>HIL Standard VTOL QuadPlane</td>
 <td>Maintainer: Lorenz Meier &lt;lorenz@px4.io&gt;<p><code>SYS_AUTOSTART</code> = 1001</p></td>
</tr>
<tr id="copter_simulation_sih_quadcopter_x">
 <td>SIH Quadcopter X</td>
 <td>Maintainer: Romain Chiappinelli &lt;romain.chiap@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 1100</p></td>
</tr>
</tbody>
</table>
</div>

### Tricopter Y+

<div class="frame_common">
<img src="../../assets/airframes/types/YPlus.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 电机1</li><li><b>电机2</b>: 电机2</li><li><b>电机3</b>: 电机3</li><li><b>舵机1</b>: 偏航角舵机</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="copter_tricopter_y+_generic_multirotor_with_tilt">
 <td>通用的带倾斜多旋翼</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 14001</p></td>
</tr>
</tbody>
</table>
</div>

## Plane

### Flying Wing

<div class="frame_common">
<img src="../../assets/airframes/types/FlyingWing.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="plane_flying_wing_generic_flying_wing">
 <td>Generic Flying Wing</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 3000</p></td>
</tr>
</tbody>
</table>
</div>

### Plane A-Tail

<div class="frame_common">
<img src="../../assets/airframes/types/PlaneATail.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 油门</li><li><b>舵机1</b>: 右副翼</li><li><b>舵机2</b>: 左副翼</li><li><b>舵机3</b>: V型尾右</li><li><b>舵机2</b>: V型尾左</li><li><b>舵机5</b>: 轮子</li><li><b>舵机6</b>: 襟翼右</li><li><b>舵机7</b>: 襟翼左</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="plane_plane_a-tail_applied_aeronautics_albatross">
 <td>Applied Aeronautics Albatross</td>
 <td>Maintainer: Andreas Antener &lt;andreas@uaventure.com&gt;<p><code>SYS_AUTOSTART</code> = 2106</p></td>
</tr>
</tbody>
</table>
</div>

### 仿真

<div class="frame_common">
<img src="../../assets/airframes/types/AirframeSimulation.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="plane_simulation_sih_plane_aert">
 <td>SIH plane AERT</td>
 <td>Maintainer: Romain Chiappinelli &lt;romain.chiap@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 1101</p></td>
</tr>
</tbody>
</table>
</div>

### Standard Plane

<div class="frame_common">
<img src="../../assets/airframes/types/Plane.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="plane_standard_plane_generic_standard_plane">
 <td>通用标准飞机</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 2100</p></td>
</tr>
</tbody>
</table>
</div>

## 无人车

### 无人车

<div class="frame_common">
<img src="../../assets/airframes/types/Rover.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="rover_rover_generic_rover_differential">
 <td>Generic Rover Differential</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 50000</p></td>
</tr>
<tr id="rover_rover_aion_robotics_r1_ugv">
 <td><a href="https://www.aionrobotics.com/r1">Aion Robotics R1 UGV</a></td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 50001</p></td>
</tr>
<tr id="rover_rover_generic_rover_ackermann">
 <td>Generic Rover Ackermann</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 51000</p></td>
</tr>
<tr id="rover_rover_axial_scx10_2_trail_honcho">
 <td><a href="https://www.axialadventure.com/product/1-10-scx10-ii-trail-honcho-4wd-rock-crawler-brushed-rtr/AXID9059.html">Axial SCX10 2 Trail Honcho</a></td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 51001</p></td>
</tr>
<tr id="rover_rover_generic_rover_mecanum">
 <td>Generic Rover Mecanum</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 52000</p></td>
</tr>
<tr id="rover_rover_generic_ground_vehicle_(deprecated)">
 <td>Generic Ground Vehicle (Deprecated)</td>
 <td><p><code>SYS_AUTOSTART</code> = 59000</p><br><b>Specific Outputs:</b><ul><li><b>电机1</b>: 油门</li><li><b>舵机1</b>: 转向</li></ul></td>
</tr>
<tr id="rover_rover_nxp_cup_car:_df_robot_gpx_(deprecated)">
 <td>NXP Cup car: DF Robot GPX (Deprecated)</td>
 <td>Maintainer: Katrin Moritz<p><code>SYS_AUTOSTART</code> = 59001</p><br><b>Specific Outputs:</b><ul><li><b>电机1</b>: 剩余轮子速度</li><li><b>舵机1</b>: 转向</li></ul></td>
</tr>
</tbody>
</table>
</div>

## Underwater Robot

### Underwater Robot

<div class="frame_common">
<img src="../../assets/airframes/types/AirframeUnknown.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="underwater_robot_underwater_robot_generic_underwater_robot">
 <td>Generic Underwater Robot</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 60000</p></td>
</tr>
<tr id="underwater_robot_underwater_robot_hippocampus_uuv_(unmanned_underwater_vehicle)">
 <td>HippoCampus UUV (Unmanned Underwater Vehicle)</td>
 <td>Maintainer: Daniel Duecker &lt;daniel.duecker@tuhh.de&gt;<p><code>SYS_AUTOSTART</code> = 60001</p></td>
</tr>
</tbody>
</table>
</div>

### Vectored 6 DOF UUV

<div class="frame_common">
<img src="../../assets/airframes/types/Vectored6DofUUV.svg"/>
<table>
 <thead>
   <tr><th>常规输出接法</th></tr>
 </thead>
 <tbody>
<tr>
 <td><ul><li><b>电机1</b>: 电机1，逆时针，船首水平右舷，逆时针桨</li><li><b>电机2</b>: 电机2，逆时针，船首水平左舷，逆时针桨</li><li><b>电机3</b>: 电机3，逆时针，船尾水平右舷，顺时针桨</li><li><b>电机4</b>: 电机4，逆时针，船尾水平左舷，顺时针桨</li><li><b>电机5</b>: 电机5，逆时针，船首垂直右舷，逆时针桨</li><li><b>电机6</b>: 电机6，逆时针，船首垂直左舷，顺时针桨</li><li><b>电机7</b>: 电机7，逆时针，船尾垂直右舷，顺时针桨</li><li><b>电机8</b>: 电机8，逆时针，船尾垂直左舷，逆时针桨</li></ul></td>
</tr>
</tbody></table>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="underwater_robot_vectored_6_dof_uuv_bluerov2_(heavy_configuration)">
 <td>BlueROV2 (Heavy Configuration)</td>
 <td>Maintainer: Thies Lennart Alff &lt;thies.lennart.alff@tuhh.de&gt;<p><code>SYS_AUTOSTART</code> = 60002</p></td>
</tr>
</tbody>
</table>
</div>

## 垂直起降

### 仿真

<div class="frame_common">
<img src="../../assets/airframes/types/AirframeSimulation.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="vtol_simulation_sih_tailsitter_duo">
 <td>SIH Tailsitter Duo</td>
 <td>Maintainer: Romain Chiappinelli &lt;romain.chiap@gmail.com&gt;<p><code>SYS_AUTOSTART</code> = 1102</p><br><b>Specific Outputs:</b><ul><li><b>电机1</b>: 右侧电机</li><li><b>电机2</b>: 左侧电机</li><li><b>舵机1</b>: 右升降副翼</li><li><b>舵机2</b>: 左升降副翼</li></ul></td>
</tr>
<tr id="vtol_simulation_sih_standard_vtol_quadplane">
 <td>SIH Standard VTOL QuadPlane</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 1103</p><br><b>Specific Outputs:</b><ul><li><b>Motor1</b>: MC motor front right</li><li><b>Motor2</b>: MC motor back left</li><li><b>Motor3</b>: MC motor front left</li><li><b>Motor4</b>: MC motor back right</li><li><b>Motor5</b>: Forward thrust motor</li><li><b>Servo1</b>: Ailerons (single channel)</li><li><b>Servo2</b>: Elevator</li><li><b>Servo3</b>: Rudder</li></ul></td>
</tr>
</tbody>
</table>
</div>

### 标准垂起固定翼

<div class="frame_common">
<img src="../../assets/airframes/types/VTOLPlane.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="vtol_standard_vtol_hil_standard_vtol_quadplane">
 <td>HIL Standard VTOL QuadPlane</td>
 <td>Maintainer: Roman Bapst &lt;roman@auterion.com&gt;<p><code>SYS_AUTOSTART</code> = 1002</p><br><b>Specific Outputs:</b><ul><li><b>Motor1</b>: MC motor front right</li><li><b>Motor2</b>: MC motor back left</li><li><b>Motor3</b>: MC motor front left</li><li><b>Motor4</b>: MC motor back right</li><li><b>Motor5</b>: Forward thrust motor</li><li><b>Servo1</b>: Aileron</li><li><b>Servo2</b>: Elevator</li><li><b>Servo3</b>: Rudder</li></ul></td>
</tr>
<tr id="vtol_standard_vtol_generic_standard_vtol">
 <td>通用标准VTOL</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 13000</p></td>
</tr>
</tbody>
</table>
</div>

### 立式垂直起落飞机

<div class="frame_common">
<img src="../../assets/airframes/types/AirframeUnknown.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="vtol_vtol_tailsitter_generic_vtol_tailsitter">
 <td>通用立式垂直起落飞机</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 13200</p></td>
</tr>
</tbody>
</table>
</div>

### VTOL Tiltrotor

<div class="frame_common">
<img src="../../assets/airframes/types/VTOLTiltRotor.svg"/>
</div>

<div class="frame_variant">
<table>
 <thead>
   <tr><th>参数名</th><th></th></tr>
 </thead>
<tbody>
<tr id="vtol_vtol_tiltrotor_generic_quadplane_vtol_tiltrotor">
 <td>Generic Quadplane VTOL Tiltrotor</td>
 <td><p><code>SYS_AUTOSTART</code> = 13030</p></td>
</tr>
<tr id="vtol_vtol_tiltrotor_generic_tiltrotor_vtol">
 <td>通用倾转旋翼机</td>
 <td>Maintainer: John Doe &lt;john@example.com&gt;<p><code>SYS_AUTOSTART</code> = 13100</p></td>
</tr>
</tbody>
</table>
</div>

