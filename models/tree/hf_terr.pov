/*
   HF terrain Generator
*/

#version 3.0
global_settings {
 // uncoment for final one
// assumed_gamma 2.2 hf_gray_16
 ambient_light 0.0
}

#include "colors.inc"
#include "textures.inc"


plane {z,16
 texture{
  wrinkles
  turbulence .1
  texture_map{
   [0.0
     pigment{
      crackle
      sine_wave
      turbulence .4
      color_map{
       [0.0 Gray20]
       [1.0 Gray]
      }
      scale .5
     }
    ]
   [1.0
     pigment{
      granite
      cubic_wave
      turbulence .6
      color_map{
       [0.0 Gray20]
       [1.0 White]
      }
      scale .5
     }
    ]
  }
  scale 75
 }
}


// Main spotlight creates crater mountain
light_source{
 -z*5
 color 1.2
}
light_source{
 -z*5
 color .75
}

camera{location <0,0,-8> right 1*x}
