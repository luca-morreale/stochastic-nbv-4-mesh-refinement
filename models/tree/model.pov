/*

    "Urban Tree"

    Jaime Vives Piqueres, (?)1999.

    IRTC 11-12-1999, "Gardens".

*/

// ### GLOBAL ###
#version 3.1;
global_settings{
ambient_light 0.1
  max_trace_level 32
}

// ### STANDARD INCLUDES ###
#debug "PARSING STANDARD AND CUSTOM INCLUDES...\n"
#include "colors.inc"
#include "textures.inc"
#include "stones.inc"
#include "woods.inc"
#include "metals.inc"
#include "skies.inc"


// ### PERSONAL INCLUDES ###
#include "i_macros.inc"
#include "i_treem.inc"
#include "i_win.inc"


// ### COMMON TEXTURES ###
#include "i_textu.inc"


// ### CONTROL VARIABLES (on/off) ###
#declare trace_street  =1;
#declare trace_pillars =1;
#declare trace_builds  =1;
#declare trace_dandw   =1;
#declare trace_tree    =1;
#declare trace_dirt    =1;
#declare trace_lamps   =1;
#declare trace_at_night=1;
#declare trace_area    =1;
#declare camera_number =1; // see camera statement at bottom


// ### COBBLESTNES STREET ###
#if (trace_street)
 #include "i_street.inc"
 object{cobble translate <   0,-4,-382>}
 object{cobble translate < 300,-4,-382>}
 object{cobbleh translate <-300,-4,-382>}
 object{cobble translate < 600,-4,-382>}
 object{cobble translate <-600,-4,-382>}
 // *** sidewalk ***
 object{sidew2 translate <   0,7,75>} // central, with the holes
 object{sidew  translate < 300,7,75>}
 object{sidew  translate <-300,7,75>}
 // *** sidewalk border ***
 object{sidewb translate <   0,7,-94>}
 object{sidewb translate < 300,7,-94>}
 object{sidewbh translate <-300,7,-94>}
 object{sidewb2 translate <   0,7,-85>}
 object{sidewb2 translate < 300,7,-85>}
 object{sidewb2h translate <-300,7,-85>} //aaaa
 // tree stones
 object{sidewb3 translate <-25,7,-65>}
 object{sidewb3 translate <-25,7,67>}
 object{sidewb3 rotate 90*y translate <46,7,5>}
 object{sidewb3 rotate 90*y translate <46-140,7,5>}
 object{terrain translate <-25,5,0>}
 object{tap translate <75,14,150>}
 //object{rear_street translate -975*z}
#end


// ### DIRTY ###
#if (trace_dirt)
 #debug "PARSING DIRTY...\n"
 // --- dog shit ---
 #include "i_dogshit.inc"
 object{shit translate <-120,15,25>}
 // --- papers ---
 #include "i_dpap.inc"
 object{papers_on_street}
 object{papers_on_sidew}
 object{papers_on_tree translate <-25,10,0>}
 // --- cigarretes ---
 #include "i_dcig.inc"
 object{cig_on_street}
 object{cig_on_sidew}
 object{cig_on_tree translate <-25,10,0>}
 // --- fallen leaves ---
 #include "i_fleaves.inc"
 object{fleaves}
#end


// ### PILLARS ###
#if (trace_pillars)
 #debug "PARSING PILLARS...\n"
 #include "i_pillars.inc"
 #declare r_pil=seed(363);
 object{pillar(t_pillar_metal,rand(r_pil)) translate <-150,14,-45>}
 object{pillar(t_pillar_metal,rand(r_pil)) translate <-210,14,-45>}
 object{pillar(t_pillar_metal,rand(r_pil)) translate <-270,14,-45>}
 object{pillar(t_pillar_metal,rand(r_pil)) translate < 100,14,-45>}
 object{pillar(t_pillar_metal,rand(r_pil)) translate < 160,14,-45>}
 object{pillar(t_pillar_metal,rand(r_pil)) translate < 220,14,-45>}
 object{pillar(t_pillar_metal,rand(r_pil)) translate < 280,14,-45>}
#end


// ### BUILDINGS ###
#if (trace_builds)
 #debug "PARSING BUILDINGS...\n"
 #include "i_builds.inc"
 // left building
 object{left_build translate <-100,375*.5+14,225>}
 // right building
 object{right_build translate <+250-100+375*.5+.5,250+14,224>}
 // cables of both buildings
 object{cables}
#end


// ### DOORS and WINDOWS ###
#if (trace_dandw)
 #debug "PARSING DOOR AND WINDOWS...\n"
 #include "i_doorswin.inc"
 object{door1 translate <250-100+375*.5+.5-50,16+d1_height*.5,259>}
 object{banner rotate -5*x translate <250-100+375*.5+.5-50,16+d1_height+16+7,219>}
 object{door2 translate <-260,16+d2_height*.5,245>}
 object{window1 translate <-100+85,14+80+v_height*.5,245>}
 object{window1 translate <-260,14+310+v_height*.5,245>}
 object{window1 translate <-100+85,14+310+v_height*.5,245>}
 plane{z,800 pigment{Flesh} hollow}
#end


// ### MOON & NIGTH SKY ###     // not translated of 375
//#if (trace_at_night)
// #debug "PARSING MOONLIGTH AND SKY...\n"
// light_source{
//  <-5000,24000,-10000>
//  Silver*.125+MidnightBlue*.075
//  fade_distance 1000000
//  fade_power 2
// }
// sky_sphere{S_Cloud3}
//#else
 // day light for tests
// light_source{
//  <-1000,5000,-2000>
//  White*.75+Gold*.5+SkyBlue*.125
//  fade_distance 10000
//  fade_power 2
// }
// sky_sphere{S_Cloud2}
//#end


// ### STREET LAMPS ###
#if (trace_lamps)
 #debug "PARSING STREET LAMPS...\n"
 #include "i_slamp.inc"
 object{slamp translate <185,250,203>}
 object{slamp_2 rotate 180*y translate <-600,250,-945>}
#end


// ### TREE ###
#if (trace_tree)
 #debug "PARSING TREE...\n"
 #include "i_otree.inc"
 object{orange_tree translate <-20,14,0>} // z = 375
#end


// ### CAMERA ###
#switch (camera_number)
 #case (1)  // +++ final +++
  #declare cam_from=<550,500,0>;
  #declare cam_zoom=2.1*z;
  #declare cam_to=<0,50,0>;
  #break
 #case (2)  // +++ to the lamp +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=7*z;
  #declare cam_to=<180,280,600>;
  #break
 #case (3)  // +++ to the door at left +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=5*z;
  #declare cam_to=<-250,140,600>;
  #break
 #case (4)  // +++ to the window +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=4*z;
  #declare cam_to=<0,140,600>;
  #break
 #case (5)  // +++ to the rigth door +++
  #declare cam_from=<100,170,-400>;
  #declare cam_zoom=3.5*z;
  #declare cam_to=<290,135,600>;
  #break
 #case (6)  // +++ to one pillar +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=9*z;
  #declare cam_to=<90,30,330>;
  #break
 #case (7)  // +++ to the balcony at left +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=3.5*z;
  #declare cam_to=<-250,320,600>;
  #break
 #case (8)  // +++ to sewer and sewer tap +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=12*z;
  #declare cam_to=<110,20,600>;
  #break
 #case (9)  // +++ to the sewer entry +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=10*z;
  #declare cam_to=<-175,10,300>;
  #break
 #case (10)  // +++ to the shit! (this sounds very funny in spanish;) +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=30*z;
  #declare cam_to=<-120,15,400>;
  #break
 #case (11)  // +++ to the tree terrain +++
  #declare cam_from=<10,170,-400>;
  #declare cam_zoom=8*z;
  #declare cam_to=<-25,15,350>;
  #break
#end

// Point of View used
#declare PdV1 = <550, 500, -300>; // +KFF10 with translate <-700*clock, 0, 0>
#declare PdV2 = <550, 300, -100>; // +KFF10 with translate <-700*clock, 0, 0>
#declare PdV3 = <550, 300, -100>; // +KFF7 with rotate <0, clock*130, 0>
#declare PdV4 = <500, 100, 150>; // +KFF10 with rotate <0, clock*130, 0>
#declare PdV5 = <350, 150, 0>; // +KFF10 with rotate <0, clock*150, 0>

#declare cam_to1 = <550, 300, 200>;
#declare cam_to2 = <550, 300, 200>;
#declare cam_to3 = <0, 150, 0>;
#declare cam_to4 = <0, 150, 150>;
#declare cam_to5 = <0, 150, 550>;

camera{
 location PdV5
 look_at 0
 angle 90
 rotate <0, clock*150, 0>
 look_at cam_to5
 //translate <-600*clock, 0, 0>
}
