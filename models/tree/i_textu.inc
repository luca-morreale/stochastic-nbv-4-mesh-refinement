// ### COMMON TEXTURES ###

// *** dirt ***
#declare t_dirty=
texture{
 pigment{
  wrinkles
  color_map{
   [0.0 Clear]
   [1.0 DarkBrown*.5 filter 0.5]
  }
 }
}
// *** rust ***
#declare t_rusty=
 texture{
  pigment{
   granite
   color_map{
    [0 DarkTan*.5]
    [0.5 DarkBrown*.5]
    [1 Orange*.6]
   }
  }
  normal{granite scale .1}
  finish{F_MetalA}
 }
// *** pillar metal ***
#declare t_metal_pillar=
 texture{
  pigment{
   wrinkles
   turbulence .3
   color_map{
    [0.00 DkGreenCopper*.1]
    [1.00 Copper]
   }
  }
  normal{crackle .1 scale .5}
  finish{Dull metallic}
 }
// *** left door metal ***
#declare t_metal_door=
 texture{
  pigment{
   wrinkles
   turbulence .3
   color_map{
    [0.00 Gray10]
    [1.00 Bronze]
   }
  }
  normal{bumps .05}
  finish{Phong_Shiny}
 }
// *** tap metal ***
#declare t_metal_tap=
 texture{
  pigment{
   wrinkles
   turbulence .3
   color_map{
    [0.00 DkGreenCopper*.1+Gray20]
    [1.00 Copper*.5+Gray10]
   }
  }
  normal{crackle scale .5}
  finish{Dull metallic}
 }
// *** pilllar metal with rust ***
#declare t_pillar_metal=
texture{
 wrinkles
 turbulence .3
 texture_map{
  [0.00 t_rusty]
  [0.30 t_rusty]
  [0.32 t_metal_pillar]
  [1.00 t_metal_pillar]
 }
 scale 4
}
// *** tap metal with rust ***
#declare t_tap_metal=
texture{
 wrinkles
 turbulence .3
 texture_map{
  [0.00 t_rusty]
  [0.30 t_rusty]
  [0.32 t_metal_tap]
  [1.00 t_metal_tap]
 }
 scale 4
}
// *** door metal with rust ***
#declare t_door_metal=
texture{
 wrinkles
 turbulence .3
 texture_map{
  [0.00 t_rusty]
  [0.10 t_rusty]
  [0.12 t_metal_door]
  [1.00 t_metal_door]
 }
 scale 4
}
// *** brick maps ***
#declare r_brick=seed(463);
#declare t_brick=
texture{
 T_Grnt4
 scale .5
}
#declare t_brick2=
texture{
 T_Grnt16
 scale .5
}
#declare t_brick_map=
 texture{
  material_map{
   gif "mm_cobble"
   texture{pigment{Flesh} normal{granite scale .1}}
   texture{t_brick translate (-50+100*rand(r_brick))*z}
   texture{t_brick translate (-50+100*rand(r_brick))*z}
   texture{t_brick translate (-50+100*rand(r_brick))*z}
   texture{t_brick translate (-50+100*rand(r_brick))*z}
   texture{t_brick translate (-50+100*rand(r_brick))*z}
  }
 }
#declare t_brick_map2=
 texture{
  material_map{
   gif "mm_cobble"
   texture{pigment{Gray80} normal{granite scale .1}}
   texture{t_brick2 translate (-50+100*rand(r_brick))*z}
   texture{t_brick2 translate (-50+100*rand(r_brick))*z}
   texture{t_brick2 translate (-50+100*rand(r_brick))*z}
   texture{t_brick2 translate (-50+100*rand(r_brick))*z}
   texture{t_brick2 translate (-50+100*rand(r_brick))*z}
  }
 }
// *** cables plastic ***
#declare t_cable=
texture{pigment{Gray10} normal{wrinkles} finish{Dull}}
texture{t_dirty}
// *** right door metal ***
#declare t_aluminum=
texture{
 T_Chrome_1A
 normal{wrinkles .1}
}
texture{t_dirty scale 10}
// *** banner textures ***
#declare t_banner_wood=
texture{T_Wood7 normal{wood} rotate 90*y scale 10}
texture{t_dirty scale 10}
#declare t_banner_paint=
texture{pigment{Red*.75+Gray*.25} normal{wood} rotate 90*y scale 10}
texture{t_dirty scale 10}
// *** left door textures ***
#declare t_d2_metal=texture{pigment{White}}
#declare t_d2_woods=texture{t_door_metal scale 4}
#declare t_d2_glass=texture{Glass normal{bumps .05 scale 4}}
// *** window textures ***
#declare t_w_metal=texture{Bronze_Metal}
#declare t_w_woods=
texture{
 T_Wood8
 normal{wood scale .1}
 finish{Dull reflection .1}
 scale 4
}
#declare t_w_glass=texture{Glass normal{bumps .05 scale 4}}
// *** cobblestone textures ***
#declare r_cobble=seed(4637);
#declare t_cobble=
texture{
 pigment{
  gradient z
  color_map{
   [0 DarkTan*.5+Gray20]
   [.5 Gray40]
   [1 DarkBrown*.5+Gray20]
  }
 }
 normal{granite scale .1}
// finish{Phong_Shiny}
 scale .5
}
#declare t_cobble_map=
 texture{
  material_map{
   gif "mm_cobble"
   texture{T_Stone3}
   texture{t_cobble translate (-50+100*rand(r_cobble))*z}
   texture{t_cobble translate (-50+100*rand(r_cobble))*z}
   texture{t_cobble translate (-50+100*rand(r_cobble))*z}
   texture{t_cobble translate (-50+100*rand(r_cobble))*z}
   texture{t_cobble translate (-50+100*rand(r_cobble))*z}
  }
 }
// *** sidewalk textures ***
#declare t_sidew=
texture{
 T_Grnt9
 scale .05
}
texture{t_dirty}
#declare t_sidewb=
texture{
 T_Grnt5
 scale .1
}
// *** tree terrain texture ***
#declare t_terrain=
texture{
 pigment{
  granite
  turbulence 0.3
  color_map{
   [0.0 DarkGreen]
   [0.1 DarkBrown]
   [0.5 DarkTan]
   [0.6 ForestGreen]
   [0.7 Tan]
   [0.9 Gray]
   [1.0 DarkGreen]
  }
 }
 normal{granite turbulence 0.3}
 scale .1
}
// *** TREE TEXTURES ****
#declare t_bark=
texture{
 pigment{
  granite
  turbulence 0.5
  color_map{
   [0.0 Tan]
   [0.4 Flesh]
   [0.5 Gray]
   [1.0 Tan]
  }
 }
 normal{granite 4 turbulence 0.5}
 scale 8
}
#declare t_leaves=
texture{
 pigment{
  gradient z
  color_map{
   [0.0 YellowGreen]
   [0.5 Goldenrod]
   [1.0 ForestGreen]
  }
  scale 20
 }
 normal{crackle scale .5}
 finish{Phong_Shiny ambient .5}
}
