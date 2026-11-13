# Inputs of the SIH terrain, enough for a viewer to rebuild it
# published by SIH at 1 Hz and on every parameter change, so a logger or viewer that starts later still gets it

uint64 timestamp        # time since system start (microseconds)

uint8 mode              # the ground in force, 0 flat, 1 terrain from elevation data, 2 synthetic terrain, as SIH_TERR_MODE numbers them
uint16 formula          # generator in the high byte, its version in the low byte, 0x0101 is the fBm terrain version 1
int32 seed              # seed of the world

float32[12] params      # inputs of the generator, their meaning set by formula
                        # for 0x0101 amplitude [m], wavelength of the largest octave [m], octaves, the rest 0
