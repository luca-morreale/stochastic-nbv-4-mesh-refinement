// *** doors and windows ***

// --- door at right ---
#declare d1_width=140;
#declare d1_height=210;
#declare d1_thick=4;
#declare door1=
union{
 #declare num_elem=29;
 #declare cnt_elem=1;
 #while (cnt_elem<=num_elem)
  union{
   cylinder{<-d1_width*.5,0,0>,<d1_width*.5,0,0>,4 scale <1,1,0.3>}
   cylinder{<-d1_width*.5,-4,-.5>,<d1_width*.5,-4,-.5>,.5}
   translate <0,4.7+8*(cnt_elem-1),0>
  }
  #declare cnt_elem=cnt_elem+1;
 #end
 cylinder{<0,5,0>,<0,5,-7>,1 translate -40*x}
 cylinder{<0,5,0>,<0,5,-7>,1 translate  40*x}
 cylinder{<0,5,0>,<0,5,2>,3}
 translate -d1_height*.5*y
 texture{t_aluminum}
}

// --- banner over the door right ---
#declare r_bann=seed(3591);
#declare banner=
union{
 box{
  <-d1_width*.6,-16,-.5>
  < d1_width*.6,  0, .5>
  texture{t_banner_wood translate 100*rand(r_bann)}
 }
 box{
  <-d1_width*.6,  0,-.5>
  < d1_width*.6, 16, .5>
  texture{t_banner_wood translate 100*rand(r_bann)}
 }
 box{
  <-d1_width*.6-2,-18,-1>
  < d1_width*.6+2,-16,.5>
  texture{t_banner_paint translate 100*rand(r_bann)}
 }
 box{
  <-d1_width*.6-2,18,-1>
  < d1_width*.6+2,16,.5>
  texture{t_banner_paint translate 100*rand(r_bann)}
 }
 box{
  <-d1_width*.6-2,-18,-1>
  <-d1_width*.6,   18,.5>
  texture{t_banner_paint rotate 90*z translate 100*rand(r_bann)}
 }
 box{
  <d1_width*.6-2,-18,-1>
  <d1_width*.6,   18,.5>
  texture{t_banner_paint rotate 90*z translate 100*rand(r_bann)}
 }
 text{
  ttf "timrom.ttf"
  "CARNICERIA"
  1,0
  scale <d1_width*.17,22,2>
  texture{t_banner_paint rotate 90*z translate 100*rand(r_bann)}
  translate <-d1_width*.55,-8,-1.5>
 }
}

// --- door left ---
#declare r=seed(543);
#declare d2_width=158;
#declare d2_height=218;
#declare d2_thick=8;
#declare l_aperture=0;
#declare r_aperture=0;
#declare use_glass=on;
#declare rnd_glass=on;
#declare door2=
union{
 object{
  Window_01(d2_width,d2_height,d2_thick,t_d2_woods,t_d2_glass,t_d2_metal,l_aperture,r_aperture,r,use_glass,rnd_glass)
 }
 union{
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+30,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+40,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+55,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+65,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+30,6,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+40,6,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+55,6,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+65,6,-d2_thick*.2>
  }
  texture{t_w_woods}
 }
 union{
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+30,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+40,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+55,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+65,-d2_height*.5+20,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+30,6,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+40,6,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+55,6,-d2_thick*.2>
  }
  cylinder{<0,0,0>,<0,85,0>,1
   translate <-d2_width*.5+65,6,-d2_thick*.2>
  }
  translate 65*x
  texture{t_w_woods}
 }
 union{
  cylinder{<0,0,0>,<0,30,0>,2 translate <-5,-10,-3>}
  cylinder{<0,0,0>,<0,30,0>,2 translate < 5,-10,-3>}
  cylinder{<0,0,0>,<0,0,1>,1 translate <4,-20,-2>}
  cylinder{<0,0,-.5>,<0,0,1>,2 translate <4,-20,-2>}
  texture{T_Chrome_2B}
 }
}

// --- window on left build ---
#declare r=seed(543);
#declare v_width=118;
#declare v_height=118;
#declare v_thick=8;
#declare l_aperture=0;
#declare r_aperture=0;
#declare use_glass=on;
#declare rnd_glass=on;
#declare v_grille=
height_field{
 png "hf_reja2"
 water_level 0.1
 translate -.5
 rotate -90*x
 scale <v_width,v_height,2>
 texture{t_door_metal}
}
#declare window1=
union{
 object{
  Window_01(v_width,v_height,v_thick,t_w_woods,t_w_glass,t_w_metal,l_aperture,r_aperture,r,use_glass,rnd_glass)
 }
 height_field{
  png "hf_pers"
  water_level 0.1
  translate -.5
  rotate -90*x
  scale <v_width,v_height,v_thick*.2>
  texture{
   pigment{Flesh}
   finish{metallic}
  }
  translate -v_thick*z
 }
 object{v_grille translate -v_thick*2*z}
}
