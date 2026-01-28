# Tailsitter VTOL

A **Tailsitter VTOL** takes off and lands on its tail, but flips into the fixed-wing orientation for normal flight.
The tailsitter rotors are permanently fixed in position for forward flight.

Tailsitters are typically mechanically less complex than [other types of VTOL](../frames_vtol/index.md), and hence may be less expensive to build and maintain.
However they are aerodynamically more complex, and can be harder to tune and fly for hover and transition, particularly in windy conditions.

## Tailsitter Airframes

:::: tabs

::: tab Duo Tailsitter
Two rotor VTOL that uses elevons to flip from hover into fixed-wing flight.

![Wingtra: WingtraOne VTOL Duo Tailsitter](../../assets/airframes/vtol/wingtraone/hero.jpg)

- Forward flight more efficient
- Hover harder to fly, particularly in wind
- Hover and transition harder to tune
- More compact form factor

:::

::: tab VTOL Tailsitter
VTOL with optional elevons. Uses rotors to transition between modes (along with elevons if present).

![Skypull SP-1 VTOL QuadTailsitter](../../assets/airframes/vtol/skypull/skypull_sp1.jpg)

- Hover mode easier to fly and more stable
- Less compact form factor (harder to transport)
- "X" and "+" rotor configurations are supported (see Airframe Reference)

:::

::::

Duo Tailsitters are generally more efficient in cruise flight (4 smaller propellers are less efficient than 2 larger ones) and physically more compact.
However as they are aerodynamically much more complex in hover mode, they are much harder to tune for both hover and transition.
Quad tailsitters are easier to fly in hover mode, and more stable in windy conditions.
Both are supported using the same aiframe type in PX4.

## Setup/Flying

VTOL setup and flying are covered in the [VTOL](../frames_vtol/index.md) parent topic.

::: info
The instructions are essentially the same for all VTOL.
The main frame-specific differences are the motor wiring, and some aspects of tuning the configuration.
:::

## Build Logs

Step-by-step guides of how to set up PX4 on tailsitter frames are listed below:

- [TBS Caipiroshka Tailsitter Build (Pixracer)](../frames_vtol/vtol_tailsitter_caipiroshka_pixracer.md)

:::tip
We recommend that you also review build logs for other PX4 VTOL, and Copter vehicles (most of the setup is the same).
:::

## Videos

This section contains videos that are specific to Tailsitter VTOL (videos that apply to all VTOL types can be found in [VTOL](../frames_vtol/index.md)).

### Duo

---

[TBS Caipiroshka](../frames_vtol/vtol_tailsitter_caipiroshka_pixracer.md) - Tailsitter takeoff (close up), hover, level flight, transitions.

<lite-youtube videoid="acG0aTuf3f8" title="PX4 VTOL - Call for Testpilots"/>

---

[Woshark](http://www.laarlab.cn/#/) _PX4 Tailsitter prototype_ - Tailsitter takeoff, transition, landing.

<!-- provided by slack user xdwgood: https://github.com/PX4/PX4-user_guide/issues/2328#issuecomment-1467234118 -->
<!-- Update issue https://github.com/PX4/PX4-user_guide/issues/3007 -->

<lite-youtube videoid="gjHj6YsxcZk" title="PX4 Autopilot Tailsitter"/>

### Quad

[UAV Works VALAQ Patrol Tailsitter](https://www.valaqpatrol.com/valaq_patrol_technical_data/) - Tailsitter takeoff, transition, landing.

<lite-youtube videoid="pWt6uoqpPIw" title="UAV Works VALAQ"/>


## Gallery

<div class="grid_wrapper three_column">
  <div class="grid_item">
    <div class="grid_item_heading"><big><a href="https://wingtra.com/mapping-drone-wingtraone/">WingtraOne</a></big></div>
    <div class="grid_text">
    <img src="../../assets/airframes/vtol/wingtraone/hero.jpg" title="Wingtra: WingtraOne VTOL Duo Tailsitter" alt="wingtraone" />
    </div>
  </div>
  <div class="grid_item">
    <div class="grid_item_heading"><big><a href="https://www.skypull.technology/">Skypull</a></big></div>
    <div class="grid_text">
      <img title="Skypull SP-1 VTOL QuadTailsitter" src="../../assets/airframes/vtol/skypull/skypull_sp1.jpg" />
    </div>
  </div>
  <div class="grid_item">
    <div class="grid_item_heading"><big><a href="../frames_vtol/vtol_tailsitter_caipiroshka_pixracer.html">TBS Caipiroshka</a></big></div>
    <div class="grid_text">
      <img title="TBS Caipiroshka" src="../../assets/airframes/vtol/caipiroshka/caipiroshka.jpg" />
    </div>
  </div>
  <div class="grid_item">
    <div class="grid_item_heading"><big><a href="http://uav-cas.ac.cn/WOSHARK/">Woshark</a></big></div>
    <div class="grid_text">
      <img title="Woshark" src="../../assets/airframes/vtol/xdwgood_ax1800/hero.jpg" />
    </div>
  </div>
  <div class="grid_item">
    <div class="grid_item_heading"><big><a href="https://www.valaqpatrol.com/valaq_patrol_technical_data/">UAV Works VALAQ Patrol Tailsitter</a></big></div>
    <div class="grid_text">
      <img title="UAV Works VALAQ Patrol Tailsitter" src="../../assets/airframes/vtol/uav_works_valaq_patrol/hero.jpg" />
    </div>
  </div>
</div>
