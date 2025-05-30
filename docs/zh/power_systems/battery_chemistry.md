# Battery-Chemistry Overview

This section provides a comparative overview of several different battery types (in particular LiPo and Li-Ion) and some useful battery glossary terms.

## LiPo vs Li-Ion

- Li-Ion batteries have a higher energy density than Lipo battery packs but that comes at the expense of lower discharge rates and increased battery cost.
- LiPo batteries are readily available and can withstand higher discharge rates that are common in multi-rotor aircraft.
- The choice needs to be made based on the vehicle and the mission being flown.
  If absolute endurance is the aim then there is more of a benefit to flying to a Li-Ion battery but similarly, more caution needs to be taken.
  As such, the decision should be made based on the factors surrounding the flight.

### LiPo

Advantages:

- Very common
- Wide range of sizes, capacities and voltages
- Inexpensive
- High discharge rates relative to capacity (high C ratings)
- Higher charge rates

缺点:

- Low (relative) energy density
- Quality can vary given abundance of suppliers

### Li-Ion

Advantages:

- Much higher energy density (up to 60% higher)

缺点:

- Not as common
- Much more expensive
- Not as widely available in large sizes and configurations
- All cells are relatively small so larger packs are made up of many cells tied in series and parallel to create the required voltage and capacity
- Lower discharge rates relative to battery size (C rating)
- More difficult to adapt to vehicles that require high currents
- Lower charging rates (relative to capacity)
- Requires more stringent temperature monitoring during charge and discharge
- Requires settings changes on the ESC to utilize max capacity ("standard" ESC low voltage settings are too high).
- At close-to-empty the voltage of the battery is such that a ~3V difference is possible between a Lipo to Li-ion (while using a 6S battery).
  This could have implications on thrust expectations.

## C Ratings

- A C rating is simply a multiple of the stated capacity of any battery type.
- A C rating is relevant (and differs) for both charge and discharge rates.
  - For example, a 2000 mAh battery (irrespective of voltage) with a 10C discharge rate can safely and continuously discharge 20 amps of current (2000/1000=2Ah x 10C = 20 amps).
- C Ratings are always given by the manufacturer (often on the outside of the battery pack).
  While they can actually be calculated, you need several pieces of information, and to measure the internal resistance of the cells.
- LiPo batteries will always have a higher C rating than a Li-Ion battery.
  This is due to chemistry type but also to the internal resistance per cell (which is due to the chemistry type) leading to higher discharge rates for LiPo batteries.
- Following manufacturer guidelines for both charge and discharge C ratings is very important for the health of your battery and to operate your vehicle safely (i.e. reduce fires, “puffing” packs and other suboptimal states during charging and discharging).

## Energy Density

- Energy density is how much energy is able to be stored relative to battery weight.
  It is generally measured and compared in Watt Hour per Kilogram (Wh/Kg).
  - Watt-hours are simply calculated by taking the nominal (i.e. not the fully charged voltage) multiplied by the capacity, e.g. 3.7v X 5 Ah = 18.5Wh.
    If you had a 3 cell battery pack your pack would be 18.5Wh X 3 = 55 Wh of stored energy.
- When you take battery weight into account you calculate energy density by taking the watt-hours and dividing them by weight.
  - E.g. 55 Wh divided by (battery weight in grams divided by 1000).
    Assuming this battery weighed 300 grams then 55/(300/1000)=185 Wh/Kg.
- This number 185 Wh/Kg would be on the very high-end for a LiPo battery.
  A Li-Ion battery on the other hand can reach 260 Wh/Kg, meaning per kilogram of battery onboard you can carry 75 more watt-hours.
  - If you know how many watts your vehicle takes to fly (which a battery current module can show you), you can equate this increased storage at no additional weight into increased flight time.
