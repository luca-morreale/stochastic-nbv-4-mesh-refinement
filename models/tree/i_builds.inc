// ### BUILDINGS (with stuff) ###

// *** door street numbers ***
#declare num_27=
height_field{
 png "hf_num27"
 translate -.5
 rotate -90*x
 texture{
  pigment{
   gradient z
   color_map{
    [0.0 NavyBlue*.5+SteelBlue*.5]
    [0.2 NavyBlue*.5+SteelBlue*.5]
    [0.2 Silver]
    [1.0 Silver]
   }
  }
  finish{Dull metallic}
 }
 texture{t_dirty scale 2}
 scale <20,15,0.5>
}

// *** dentist plaque ***
#declare plaque=
height_field{
 png "hf_plaque"
 translate -.5
 rotate -90*x
 texture{pigment{Brass} finish{Dull metallic reflection .3}}
 texture{t_dirty scale 2 translate 10}
 scale <36,27,2>
}

// *** balcony grille ***
#declare grille=
height_field{
 png "hf_reja"
 water_level 0.1
 translate -.5
 rotate -90*x
 scale <150,100,2>
 texture{t_pillar_metal}
}

// *** door lamp ***
#declare perilla=
union{
 // *** direct light from the bulb ***
 light_source{
  <0,0,0>
  Gold*.5+Pink*.4
  spotlight
  radius 20
  falloff 40
  tightness 1
  point_at <0, -225, 0>
  fade_distance 200
  fade_power 4
 }
 // faked ambient
/*
 light_source{
  <0,-2,0>
  Gold*.125+Pink*.1
  fade_distance 200
  fade_power 4
 }
 */
 intersection{
  sphere{<0,0,0>,1}
  plane{y,0}
    pigment { color rgbf<1, 1, 1, 1> }
    finish { ambient 0 diffuse 0 }
    interior{
     media {
      emission 0.075
      intervals 10
      samples 1, 10
      confidence 0.9999
      variance 1/1000
      density {
        spherical
        turbulence .1
        color_map {
          [0.0 Black]
          [1.0 Pink+Gold]
        }
        frequency 1
      }
     }
    }
    no_shadow
    hollow
    scale 10
 }
}

// *** rounded box for balcony ***
#declare rbox=
superellipsoid{
 <.05,.2>
 scale .5
 rotate -90*x
}
box{-.5,.5}

// *** wall of bricks ***
#declare brick_wall=
height_field{
 png "hf_cobble"
 translate -.5
 rotate -90*x
}

// *** left building ***
#declare left_build=
union{
 // *** wall ***
 object{brick_wall
  scale <500,375,4>
  texture{
   t_brick_map
   translate -.5
   scale <500,375,4>
  }
  translate -500*x
 }
 difference{
  object{brick_wall scale <500,375,4>}
  // *** holes for the door and windows ***
  box{<-80,-375*.5,-10>,<80,-375*.5+222,790> translate -160*x}
  box{<-60,-375*.5+80,-10>,<60,-375*.5+80+120,790> translate 85*x}
  box{<-60,-375*.5+310,-10>,<60,375*.49,790> translate -160*x}
  box{<-60,-375*.5+310,-10>,<60,375*.49,790> translate 85*x}
  texture{
   t_brick_map
   translate -.5
   scale <500,375,4>
  }
 }
 // *** floor separator ***
 object{rbox
  scale <500,10,15>
  translate <0,5,2.5>
  translate (-375*.5+270)*y
  texture{pigment{Flesh} normal{granite scale 2}}
 }
 // *** door ceil ***
 object{rbox
  scale <200,15,70>
  translate <0,17.5,-25>
  translate <-160,-375*.5+223,0>
  texture{pigment{Flesh} normal{granite scale 3}}
 }
 union{
  difference{
   box{<-90,0,-40>,<90,10,10>}
   cylinder{<0,-1,-5>,<0,8,-5>,3}
   texture{T_Wood7 rotate 90*y scale 10}
  }
  object{perilla rotate -8*x translate <0,0,-5>}
  translate <-160,-375*.5+223,0>
 }
 // *** balcony ***
 union{
  // *** left ***
  union{
   box{<-60,0,-30>,<60,5,10>}
   box{<-65,5,-35>,<65,10,10>}
   box{<-70,10,-40>,<70,15,10>}
   box{<-75,15,-45>,<75,20,10>}
   object{rbox
    scale <160,10,60>
    translate <0,25,-20>
   }
   texture{pigment{Flesh} normal{granite scale 2}}
   translate <85,-375*.5+280,0>
  }
  union{
   object{grille}
   object{grille rotate  90*y translate < 74,0,74>}
   object{grille rotate -90*y translate <-74,0,74>}
   translate <85,-375*.5+311+50,-45>
  }
  // *** right ***
  union{
   box{<-60,0,-30>,<60,5,10>}
   box{<-65,5,-35>,<65,10,10>}
   box{<-70,10,-40>,<70,15,10>}
   box{<-75,15,-45>,<75,20,10>}
   object{rbox
    scale <160,10,60>
    translate <0,25,-20>
   }
   texture{pigment{Flesh} normal{granite scale 2}}
   translate <-160,-375*.5+280,0>
  }
  union{
   object{grille}
   object{grille rotate  90*y translate < 74,0,74>}
   object{grille rotate -90*y translate <-74,0,74>}
   translate <-160,-375*.5+311+50,-45>
  }
 }
 // *** window stone ***
 box{<-62,0,-6>,<62,4,20>
  texture{T_Grnt3 scale 10}
  translate <85,-375*.5+77,0>
 }
 // *** door stone ***
 box{<-81,0,-6>,<81,4,50>
  texture{T_Grnt3 scale 10}
  translate <-160,-375*.5,0>
 }
 // *** sewer ***
 union{
  cylinder{<0,10,0>,<0,250,0>,6}
  cylinder{<0,250,0>,<0,260,-10>,6}
  sphere{<0,260,-10>,6}
  cylinder{<0,260,-10>,<0,280,-10>,6}
  sphere{<0,280,-10>,6}
  cylinder{<0,280,-10>,<0,290,0>,6}
  cylinder{<0,290,0>,<0,400,0>,6}
  cylinder{<0,110,0>,<0,120,0>,7.5}
  cylinder{<0,230,0>,<0,240,0>,7.5}
  cylinder{<0,265,-10>,<0,275,-10>,7.5}
  cylinder{<0,300,0>,<0,310,0>,7.5}
  cylinder{<0,10,0>,<0,20,0>,7.5}
  intersection{
   difference{
    torus{8,6 rotate <-90,90,0>}
    torus{8,5.8 rotate <-90,90,0>}
   }
   plane{y,0}
   plane{z,0 inverse}
   translate <0,10,-8>
  }
  texture{
   pigment{White}
   normal{crackle scale .1}
   finish{Phong_Shiny}
  }
  translate <243,-375*.5+10,-6>
 }
 // *** door number ***
 object{num_27 translate <-160+96,-375*.5+205,-2>}
 // *** dentist plaque ***
 object{plaque translate <-160+96+8,-375*.5+170,-1.5>}
}

// *** right building ***
#declare right_build=
union{
 // *** wall ***
 difference{
  object{brick_wall scale <375,500,4>}
  // *** holes for the door and windows ***
  box{<-70,-250,-10>,<70,-38,790> translate <-50,0,0>}
  box{<-60,-250+330,-10>,<60,249,790> translate <-50,0,0>}
  texture{
   t_brick_map2
   translate -.5
   scale <375,500,4>
  }
 }
 // *** floor separator ***
 object{rbox
  scale <375,10,15>
  translate <0,5,2.5>
  texture{pigment{Gray80} normal{granite scale 2}}
  translate (-250+280)*y
 }
 // *** balcony ***
 union{
  union{
   box{<-60,0,-20>,<-30,10,10>}
   box{<-20,0,-20>,<20,10,10>}
   box{< 30,0,-20>,<60,10,10>}
   box{<-70,10,-32>,<70,12,10>}
   box{<-72,12,-34>,<72,14,10>}
   box{<-74,14,-36>,<74,16,10>}
   box{<-76,16,-38>,<76,18,10>}
   box{<-78,18,-40>,<78,20,10>}
   object{rbox
    scale <160,10,60>
    translate <0,25,-20>
   }
   texture{pigment{Gray80} normal{granite scale 2}}
   translate <-50,-250+290,0>
  }
  union{
   object{grille}
   object{grille rotate  90*y translate < 74,0,74>}
   object{grille rotate -90*y translate <-74,0,74>}
   translate <-50,-250+321+50,-45>
  }
 }
 // *** door stone ***
 box{<-71,0,-6>,<71,4,20>
  texture{T_Grnt9 scale 10}
  translate <-50,-250,0>
 }
}

// *** cables along both buildings ***
#declare cables=
union{
 #declare sections=8;
 #declare pt1=<-330,279,598>;
 #declare pt11=<-260,279,598>;
 #declare pt2=<-190,279,598>;
 #declare pt21=<-110,279,598>;
 #declare pt3=<-30,279,598>;
 #declare pt31=<40,279,598>;
 #declare pt4=< 110,278,598>;
 #declare pt5=< 170,288,595>;
 #declare pt51=< 240,288,595>;
 #declare pt6=< 310,288,595>;
 #declare diam=3;
 #declare rcurv=3;
 object{hanging_cable(pt1,pt11,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt11,pt2,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt2,pt21,diam,sections,rcurv*1.1,t_cable)}
 object{hanging_cable(pt21,pt3,diam,sections,rcurv*1.1,t_cable)}
 object{hanging_cable(pt3,pt31,diam,sections,rcurv*1.2,t_cable)}
 object{hanging_cable(pt31,pt4,diam,sections,rcurv*1.2,t_cable)}
 object{hanging_cable(pt5,pt51,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt51,pt6,diam,sections,rcurv,t_cable)}
 sphere{pt1,2 texture{t_pillar_metal}}
 sphere{pt11,2 texture{t_pillar_metal}}
 sphere{pt2,2 texture{t_pillar_metal}}
 sphere{pt21,2 texture{t_pillar_metal}}
 sphere{pt3,2 texture{t_pillar_metal}}
 sphere{pt31,2 texture{t_pillar_metal}}
 sphere{pt4,2 texture{t_pillar_metal}}
 sphere{pt5,2 texture{t_pillar_metal}}
 sphere{pt51,2 texture{t_pillar_metal}}
 sphere{pt6,2 texture{t_pillar_metal}}
 #declare sections=8;
 #declare pt1b=<-330,277,598>;
 #declare pt1b1=<-260,277,598>;
 #declare pt2b=<-190,277,598>;
 #declare pt2b1=<-110,277,598>;
 #declare pt3b=<-30,277,598>;
 #declare pt3b1=<40,277,598>;
 #declare pt4b=< 110,275,598>;
 #declare pt5b=< 170,284,595>;
 #declare pt5b1=< 240,285,595>;
 #declare pt6b=< 310,286,595>;
 #declare diam=2;
 #declare rcurv=5;
 object{hanging_cable(pt1b,pt1b1,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt1b1,pt2b,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt2b,pt2b1,diam,sections,rcurv*1.1,t_cable)}
 object{hanging_cable(pt2b1,pt3b,diam,sections,rcurv*1.1,t_cable)}
 object{hanging_cable(pt3b,pt3b1,diam,sections,rcurv*1.2,t_cable)}
 object{hanging_cable(pt3b1,pt4b,diam,sections,rcurv*2,t_cable)}
 object{hanging_cable(pt5b,pt5b1,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt5b1,pt6b,diam,sections,rcurv,t_cable)}
 sphere{pt1b,2 texture{t_pillar_metal}}
 sphere{pt1b1,2 texture{t_pillar_metal}}
 sphere{pt2b,2 texture{t_pillar_metal}}
 sphere{pt2b1,2 texture{t_pillar_metal}}
 sphere{pt3b,2 texture{t_pillar_metal}}
 sphere{pt3b1,2 texture{t_pillar_metal}}
 sphere{pt4b,2 texture{t_pillar_metal}}
 sphere{pt5b,2 texture{t_pillar_metal}}
 sphere{pt5b1,2 texture{t_pillar_metal}}
 sphere{pt6b,2 texture{t_pillar_metal}}
 // *** electric boxes ***
 superellipsoid{
  <.2,.2>
  rotate -90*x
  texture{t_pillar_metal}
  scale <10,15,2>
  translate pt4+<10,-11,-1>
 }
 superellipsoid{
  <.2,.2>
  rotate -90*x
  texture{t_pillar_metal translate 30}
  scale <10,15,2>
  translate pt5+<-10,-11,-1>
 }
 // *** cable between boxes ***
 object{hanging_cable(pt4+<20,0,0>,pt5-<20,0,0>,diam,sections,rcurv,t_cable)}
 object{hanging_cable(pt4b+<20,-2,0>,pt5b-<20,-2,0>,diam,sections,rcurv,t_cable)}
 // *** cable to the streelamp ***
 #declare rcurv=2;
 #declare diam=1;
 object{hanging_cable(pt5b+<-16,-15,1>,pt5b+<-16,-50,1>,diam,sections,rcurv,t_cable)}
 sphere{pt5b+<-16,-50,1>,1.2 texture{t_pillar_metal}}
 #declare rcurv=3;
 object{hanging_cable(pt5b+<-16,-50,1>,pt5b+<11,-50,1>,diam,sections,rcurv,t_cable)}
 sphere{pt5b+<11,-50,1>,1.2 texture{t_pillar_metal}}
}
