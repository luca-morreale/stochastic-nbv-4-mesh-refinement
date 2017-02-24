// =========================================
// Demo file for a building front and a Mini Cooper model
// -----------------------------------------
// Made for Persistence of vision 3.6
//==========================================  
// Copyright 2004 Gilles Tran http://www.oyonale.com
// -----------------------------------------
// This work is licensed under the Creative Commons Attribution License. 
// To view a copy of this license, visit http://creativecommons.org/licenses/by/2.0/ 
// or send a letter to Creative Commons, 559 Nathan Abbott Way, Stanford, California 94305, USA.
// You are free:
// - to copy, distribute, display, and perform the work
// - to make derivative works
// - to make commercial use of the work
// Under the following conditions:
// - Attribution. You must give the original author credit.
// - For any reuse or distribution, you must make clear to others the license terms of this work.
// - Any of these conditions can be waived if you get permission from the copyright holder.
// Your fair use and other rights are in no way affected by the above. 
//==========================================  
// This picture can be rendered in one or two passes
// For the 2-pass method, render first without the reflections (Ref=0), 
// without media (MediaOK=0), without area_lights (AreaOK=0)
// without saving the file, without aa
// and save the radiosity data (SaveRadOK=1, LoadRadOK=0)
// You can also use a lower size (1/2 for instance)
// When the first pass is completed, render again, now with reflections (Ref=1),
// with media (MediaOK=1), without area_lights (AreaOK=1),
// with aa, with file output and at final size
// and of course load the radiosity data  (SaveRadOK=0, LoadRadOK=1)
// The 2-pass method should save some render time and RAM use.
// Use a better anti-aliasing than the default one
// Media is really not necessary.
//-----------------------------------------
// 2-pass render times at 600x600, PIII 1.7, 1Gb RAM, Windows XP
// Pass 1 : 28 mn
// Pass 2 : 5h08 mn, 408 Mb RAM used
//-----------------------------------------
#include "colors.inc"
#include "functions.inc"
#include "textures.inc"
#include "transforms.inc"
//---------------------------------------
// Switches (the value 0 turns a switch off)
//---------------------------------------
#declare RadOK=2; // 0 = no rad, 1 = test rad, 2 = final rad
#declare MediaOK=1; // media = 1 creates a subtle haze
#declare SaveRadOK=0; // save radiosity data for pass 1
#declare LoadRadOK=0; // load radiosity data for pass 2
#declare AreaOK=1; // subtle area light
#declare Ref=1; // intensity of reflections, use 0 to turn off it completely

#declare BuildingOK=1; // building
#declare SidewalkOK=1; // sidewalk and road
#declare MiniOK=1; // Mini Cooper car

//---------------------------------------
// Settings
//---------------------------------------
#default {finish {ambient 0}}
global_settings {
    max_trace_level 20
    //---------------------------------------
    // change gamma if necessary (scene too bright for instance)
    //---------------------------------------
    assumed_gamma 1
    //---------------------------------------
    noise_generator 2
    #if (RadOK>0)
        radiosity{
            #switch (RadOK)
                #case (1)
                    count 35 error_bound 1.8 
                #break
                #case (2)
                    count 300 error_bound 0.1 
                #break
            #end    
            nearest_count 2 
            recursion_limit 1  
            low_error_factor 1 
            gray_threshold 0 
            minimum_reuse 0.015 
            brightness 2 
            adc_bailout 0.01/2      
            normal on
            media off
            #if (SaveRadOK=1) // saves the radiosity values in the first pass
                save_file "mini.rad"
            #else
                #if (LoadRadOK=1) // loads the radiosity values in the second pass
                    pretrace_start 1
                    pretrace_end 1
                    load_file "mini.rad"
                    always_sample off
                #end
            #end
        }
    #end
}

//---------------------------------------
// Camera
//---------------------------------------
// See check comments to see the settings
#declare PdV1 = <13, 8, -10>;   // KFF15 clock*120
#declare PdV2 = <15, 10, -5>;   // KFF10 clock*120

#declare car_location=<0, 0.02, -3>;
#declare cam_location= PdV1;
#declare cam_lookat= <0, 5, 0>;

camera {
    location  cam_location
    direction z*1.5
    right     x*16/9
    look_at   cam_lookat
    angle 60
    rotate <0, clock*120, 0>
}

//---------------------------------------
// Sun
//---------------------------------------
#macro GammaColor(COLOR,G,L)
    rgb <pow(COLOR.red,G),pow(COLOR.green,G),pow(COLOR.blue,G)>*L
#end                         
#declare C_Sun  = GammaColor(<255,216,192>/255,0.4,1);
#declare xSun=60;
#declare ySun=79;
#declare posSun=vaxis_rotate(vaxis_rotate(-z*10000,x,xSun),y,ySun);

light_source {
    posSun
    C_Sun*3
    #if (AreaOK=1) area_light 100*z,100*y,4,4 jitter adaptive 1 orient circular #end
}

//---------------------------------------
// Sky
//---------------------------------------
#declare C_Sky=rgb <87,114,165>/165;

box{-0.99,0.99
    texture{
        pigment{
            pigment_pattern{
                function {min(1,max(0,y))}
                poly_wave 0.6
                scale <1/3,1,1/3>
                turbulence 0.2
                lambda 2
            }
            pigment_map{ // creates some sort of skyline, mostly useless
                [0 Black]
                [0.15 average
                    pigment_map{
                        [1 cells scale <1/30,1/2,1/30>*2 color_map{[0.5 Black][0.5 C_Sky]}]
                        [1 cells scale <1/40,1/4,1/40>*2 color_map{[0.5 Black][0.5  C_Sky]}]
                    }
                ]
                [0.15 color C_Sky]
                [1 rgb 0.5*<88,120,157>/255]
            }
            
        }
        finish{ambient 2 diffuse 0}
    }
    hollow
    scale 1000
    rotate y*80
    no_shadow
}

//---------------------------------------
// Scattering media to create some haze effect
//---------------------------------------
#if (MediaOK)
    box{
        <-100,-1,-10>, <100,30,10>
        texture{pigment{Clear}finish{ambient 0 diffuse 0}}
        hollow
        interior{
            media{
                scattering{2,0.002 extinction 1}
            }
        }
    }
#end 


#declare T_Clear=texture{pigment{Clear} finish{ambient 0 diffuse 0}}

//---------------------------------------
// Buildings
//---------------------------------------
union{
    //---------------------------------------
    // Sidewalk and road
    //---------------------------------------
    #if (SidewalkOK=1)
        #debug "sidewalk\n"
        #declare P_Road=pigment{image_map{jpeg "bd_road2" interpolate 2} rotate z*90} 
        #declare F_Road=finish{ambient 0 diffuse 0.7 specular 0.1 roughness 0.02}// reflection{0,0.2*Ref}}

        #declare P_Concrete=pigment{image_map{jpeg "bd_concrete2" interpolate 2}} 
        #declare N_Concrete=normal{bump_map{jpeg "bd_concrete_bump" interpolate 2} bump_size 0.5}
        #declare F_Concrete=finish{ambient 0 diffuse 0.5 specular 0.3 roughness 0.15}
        //---------------------------------------
        // sidewalk
        //---------------------------------------
        #declare T_sidewalk_mat=texture{
            pigment{P_Concrete}
            normal{N_Concrete}
            finish{F_Concrete}
            scale <2453/1573,1,1>*2/3
        }
        #include "bd_sidewalk_o.inc"
        #declare Sidewalk=object{ P_Figure_1 }

        //---------------------------------------
        // cover
        //---------------------------------------
        #declare T_cover_mat=texture{
            pigment{image_map{jpeg "bd_cover" interpolate 2} rotate z*90}
            normal{bump_map{jpeg "bd_cover_bmp" interpolate 2} rotate z*90}
            finish{ambient 0 diffuse 0.4 specular 0.1 roughness 1/20}
        }
        #include "bd_cover_o.inc"
        #declare Cover=object{ P_Figure_1 }

        //---------------------------------------
        // border
        //---------------------------------------
        #declare C_Border=rgb <1,0.8,0.02>;
        #declare F_Border=finish{ambient 0 diffuse 0.5 specular 0.1 roughness 0.15}
        #declare P_Border=pigment{P_Concrete scale <2453/1573,1,1>*2 rotate x*45}
        #declare T_border_mat = texture{
            pigment_pattern{
                gradient x 
                scale 6 
                triangle_wave 
                color_map{[0.46 Black][0.5 White]}
                translate x*-0.31
            }
            texture_map{
                [0 pigment{P_Border}normal{granite 0.2 scale 1/3}finish{F_Border}]
                [1 pigment{
                        average
                        pigment_map{
                            [1 wrinkles turbulence 0.1 pigment_map{[0 color C_Border*1.3][0.45 color C_Border*0.5][0.68 color C_Border][0.69 P_Border]}]
                            [1 P_Border]
                        }
                    }
                    normal{granite 0.1 scale 1/3}
                    finish{F_Border}
                ]
            }
        }
        #include "bd_border_o.inc"
        #declare Border=object{ P_def_obj }
        
        //---------------------------------------
        // road
        //---------------------------------------
        // the road is made with a tileable height_field
        
        #declare RoadUnit=height_field{
            jpeg "bd_road_hf3"
            smooth
            texture{
                pigment{P_Road}
                finish{F_Road}
                rotate x*90
            }
        }
        
        #declare Road=union{
            object{RoadUnit}
            object{RoadUnit translate z}
            object{RoadUnit translate z*2}
            object{RoadUnit translate z*3}
            object{RoadUnit translate z*4}
            union{
                object{RoadUnit}
                object{RoadUnit translate z}
                object{RoadUnit translate z*2}
                object{RoadUnit translate z*3}
                object{RoadUnit translate z*4}
                translate x
            }
            union{
                object{RoadUnit}
                object{RoadUnit translate z}
                object{RoadUnit translate z*2}
                object{RoadUnit translate z*3}
                object{RoadUnit translate z*4}
                translate x*2
            }
            scale <174/237,0.025/3,1>*3
            rotate y*90
            translate <-15,0,6>
        }

        //---------------------------------------
        // Place the sidewalk and road
        //---------------------------------------
        #declare Ground=union{
            union{
                object{Cover}
                object{Sidewalk}
                object{Border}
                rotate x*-90
                scale 4/12.649749
                scale <-1,1,1>
                translate -z*3.01+y*0.14
            }
            object{Road}
        }
        object{Ground rotate y*180 translate <-6.5, 0, -2>}
    #end

    //---------------------------------------
    // Building
    //---------------------------------------

    #if (BuildingOK=1)
        #debug "building\n"
        #declare B=-1; // general bump value
        
        #declare T_Glass = texture{
            pigment{rgbf <0.98,0.95,0.9,0.9>}
            normal{bumps -0.005 scale 1} 
            finish{ambient 0 diffuse 0.45 specular 1 roughness 1/1000 conserve_energy reflection{0.3*Ref,0.9*Ref fresnel on}}
        }
        #declare C_Wall=rgb <115,110,94>/255;
        #declare C_Wall=GammaColor(C_Wall,2,1.5);
        #declare T_Wall0=texture{pigment{rgbt <C_Wall.red,C_Wall.green,C_Wall.blue,0.7>} finish{ambient 0 diffuse 1}}
        #declare T_Wall=texture{pigment{White*0.5+Yellow*0.5}}
        #declare T_Window=texture{
            pigment{image_map{jpeg "bd_window" interpolate 2}} 
            finish{ambient 0 specular 0.3 roughness 1/30 diffuse 0.7}
        }
        #declare T_DEFAULT = texture{T_Wall}
        
        
        #declare T_def_mat=texture{pigment{image_map{jpeg "bd_corner2" interpolate 2}} normal{bump_map{jpeg "bd_corner2_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_corner2_o.inc"
        #declare P_corner2=object{ P_Figure_1 }
        
        // columns have to be reloaded to get different textures...
        #declare T_column2_mat=texture{pigment{image_map{jpeg "bd_column2" interpolate 2}} normal{bump_map{jpeg "bd_column2_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column2_o.inc"
        #declare P_column2=object{ P_Figure_1 }
        
        #declare T_column2_mat=texture{pigment{image_map{jpeg "bd_column2b" interpolate 2}} normal{bump_map{jpeg "bd_column2b_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column2_o.inc"
        #declare P_column2b=object{ P_Figure_1 }

        #declare T_column2_mat=texture{pigment{image_map{jpeg "bd_column2c" interpolate 2}} normal{bump_map{jpeg "bd_column2c_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column2_o.inc"
        #declare P_column2c=object{ P_Figure_1 }

        #declare T_ledge2w_mat=texture{pigment{image_map{jpeg "bd_ledge2w" interpolate 2}} normal{bump_map{jpeg "bd_ledge2w_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_ledge2w_o.inc"
        #declare P_ledge2w=object{ P_Figure_1 }
        
        #declare T_ledge2_mat=texture{pigment{image_map{jpeg "bd_ledge2" interpolate 2} scale <1/6,1,1>*2} normal{bump_map{jpeg "bd_ledge2_bump" interpolate 2} bump_size B scale <1/6,1,1>*2} finish{ambient 0 diffuse 0.8}}texture{T_Wall0}
        #include "bd_ledge2_o.inc"
        #declare P_ledge2=object{ P_Figure_1 }
        
        #declare T_corner1b_mat=texture{pigment{image_map{jpeg "bd_corner1b" interpolate 2} scale <1,1,1>} normal{bump_map{jpeg "bd_corner1b_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_corner1b_o.inc"
        #declare P_corner1b=object{ P_Figure_1 }
        
        #declare T_front1a_mat=texture{pigment{image_map{jpeg "bd_front1a" interpolate 2} scale <1,1,1>} normal{bump_map{jpeg "bd_front1a_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_front1a_o.inc"
        #declare P_front1a=object{ P_Figure_1 }
        
        #declare T_corner1a_mat=texture{pigment{image_map{jpeg "bd_front1" interpolate 2} scale <1,0.5,1>} normal{bump_map{jpeg "bd_front1_bump" interpolate 2} bump_size B/2 scale <1,0.5,1>} finish{ambient 0 diffuse 1}}
        #include "bd_corner1a_o.inc"
        #declare P_corner1a=object{ P_Figure_1  }
        
        #declare T_corner1_mat=texture{pigment{image_map{jpeg "bd_corner1" interpolate 2} scale <1,1,1>} normal{bump_map{jpeg "bd_corner1_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_corner1_o.inc"
        #declare P_corner1=object{ P_Figure_1  }
        
        #declare T_front1_mat=texture{pigment{image_map{jpeg "bd_front1" interpolate 2} scale <1,1,1>} normal{bump_map{jpeg "bd_front1_bump" interpolate 2} bump_size B/2} finish{ambient 0 diffuse 1}}
        #include "bd_front1_o.inc"
        #declare P_front1=object{ P_Figure_1 }
        
        // columns and bricks have to be reloaded to get different textures...
        #declare T_column1_mat=texture{pigment{image_map{jpeg "bd_column1" interpolate 2}} normal{bump_map{jpeg "bd_column1_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column1_o.inc"
        #declare P_column1=object{ P_Figure_1 }
        
        #declare T_column1_mat=texture{pigment{image_map{jpeg "bd_column1b" interpolate 2}} normal{bump_map{jpeg "bd_column1b_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column1_o.inc"
        #declare P_column1b=object{ P_Figure_1 }
        
        #declare T_column1_mat=texture{pigment{image_map{jpeg "bd_column1c" interpolate 2}} normal{bump_map{jpeg "bd_column1c_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column1_o.inc"
        #declare P_column1c=object{ P_Figure_1 }
        
        #declare T_column1_mat=texture{pigment{image_map{jpeg "bd_column1d" interpolate 2}} normal{bump_map{jpeg "bd_column1d_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column1_o.inc"
        #declare P_column1d=object{ P_Figure_1 }
        
        #declare T_bricks1_mat=texture{pigment{image_map{jpeg "bd_bricks1" interpolate 2}} normal{bump_map{jpeg "bd_bricks1_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_bricks1_o.inc"
        #declare P_bricks1=object{ P_Figure_1 }
        
        #declare T_bricks1_mat=texture{pigment{image_map{jpeg "bd_bricks2" interpolate 2}} normal{bump_map{jpeg "bd_bricks2_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_bricks1_o.inc"
        #declare P_bricks2=object{ P_Figure_1 }
        
        #declare T_dtop1a_mat = texture{pigment{image_map{jpeg "bd_doortop" interpolate 2}} normal{bump_map{jpeg "bd_doortop_bump" interpolate 2} bump_size B} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #declare T_dtop1b_mat = texture{T_dtop1a_mat}texture{T_Wall0}
        #declare T_dtop1c_mat = texture{T_dtop1a_mat}texture{T_Wall0}
        #declare T_dtop1d_mat = texture{T_dtop1a_mat}texture{T_Wall0}
        
        #include "bd_doortop_o.inc"
        
        #declare P_doortop=object{ P_Figure_1 }
        
        #declare T_dframe_mat = texture{T_front1_mat}
        
        #include "bd_dframe_o.inc"
        
        #declare T_glass1d_mat = texture{T_Glass} 
        #declare T_glass1_mat = texture{T_Glass}
        #declare T_glass2_mat = texture{T_Glass}
        #include "bd_glass_o.inc"

        // the window models have to be reloaded to get a different texture each time
        #declare T_window1_mat = texture{T_Window}
        #include "bd_window1_o.inc"
        #declare P_window1_left=object{P_def_obj}

        #declare T_window1_mat = texture{T_Window rotate z*90}
        #include "bd_window1_o.inc"
        #declare P_window1_right=object{P_def_obj}

        #declare T_window2_mat = texture{T_Window}
        #include "bd_window2_o.inc"
        #declare P_window2_1=object{P_def_obj}

        #declare T_window2_mat = texture{T_Window rotate z*90}
        #include "bd_window2_o.inc"
        #declare P_window2_2=object{P_def_obj}

        #declare T_window2_mat = texture{T_Window rotate z*180}
        #include "bd_window2_o.inc"
        #declare P_window2_3=object{P_def_obj}

        #declare T_window2_mat = texture{T_Window rotate z*270}
        #include "bd_window2_o.inc"
        #declare P_window2_4=object{P_def_obj}

        #declare T_colside1_mat=texture{pigment{image_map{jpeg "bd_colside1" interpolate 2}} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_colside1_o.inc"
        #declare P_colside1=object{ P_Figure_1 }

        #declare T_window1d_mat = texture{T_Window}
        #include "bd_window1d_o.inc"

        #declare T_column1d_mat=texture{pigment{image_map{jpeg "bd_column1door" interpolate 2}} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_column1door_o.inc"
        #declare P_column1door=object{ P_Figure_1 }
        #declare T_door1a_mat = texture{pigment{image_map{jpeg "bd_door1" interpolate 2}} finish{ambient 0 diffuse 1} scale <-1,1,1>}
        #declare T_doorbox_mat =texture{pigment{White*0.8} normal{bumps 0.3} finish{ambient 0 diffuse 0.4 metallic brilliance 2 reflection 0.3*Ref}}
        #declare T_door1b_mat = texture{T_doorbox_mat}
        #include "bd_door1_o.inc"
        #declare P_door1=object{ P_Figure_1 }
        
        #declare T_door2a_mat = texture{pigment{image_map{jpeg "bd_door2" interpolate 2}} finish{ambient 0 diffuse 1}}

        #declare P_metaldoor=pigment{image_map{jpeg "bd_door2d" interpolate 2}}
        #declare T_metaldoor_mat = 
            texture{
                    pigment{P_metaldoor}
                    finish{ambient 0 diffuse 1} 
         }
        #declare T_door2b_mat = texture{T_metaldoor_mat}        
        #include "bd_door2_o.inc"
        #declare P_door2=object{ P_Figure_1 }
        
        #declare T_num1_mat = texture{T_front1_mat finish{diffuse 0.4}}
        #declare T_num2_mat = texture{T_front1_mat finish{diffuse 0.4}}
        #declare T_plate_mat = texture{T_front1_mat}
        #declare T_numcirc_mat = texture{T_front1_mat}
        #include "bd_number_o.inc"
        #declare P_number=object{ P_Figure_1 }
        
        #declare T_mdoortop_mesh_mat = texture{T_front1_mat}
        #include "bd_mdoortop_o.inc"
        #declare P_mdoortop=object{ P_Figure_1 }
        
        #include "bd_mdoor_o.inc"
        #declare P_mdoor=object{ P_Figure_1 }
        
        #declare T_fledge_mat = texture{pigment{image_map{jpeg "bd_mdoorledge" interpolate 2}} finish{ambient 0 diffuse 1}}texture{T_Wall0}
        #include "bd_mdoorledge_o.inc"
        #declare P_mdoorledge=object{ P_Figure_1 }
        
        #declare T_curtain_mat = texture{pigment{rgb <1,0.95,0.94>}finish{ambient 0 diffuse 1}}
        #include "bd_curtain_o.inc"

        #declare Building=union{
            object{ P_glass1d }
            object{ P_door1 }
            object{ P_door2 }
            object{ P_number }
            object{ P_mdoorledge }
            object{ P_mdoor translate <-0.05,-0.35,0>*12.649749/4}
            object{ P_dframe }
            object{ P_column1door }
            object{ P_mdoortop }
        
            object{ P_doortop }
            object{ P_window1d }

            object{P_curtain translate -z*1.1*12.649749/4}
            object{ P_colside1 }
            
            #if (Ref=1) // excludes the glass panes from first pass
                object{ P_glass1}
            #end
            object{ P_window1_left }
            object{ P_bricks1 }
            object{ P_column1 }
            object{ P_corner1 }
            object{ P_front1 }
            object{ P_corner1a }
            object{ P_corner1b }
            object{ P_front1a }
            object{ P_ledge2 }
            object{ P_ledge2w }
            object{ P_window2_1 }
            
            #if (Ref=1) // excludes the glass panes from first pass
                object{ P_glass2}
            #end

            object{ P_column2 }
            object{ P_corner2 }
        
            object{ P_column2b translate x*10}
            object{ P_column2c translate x*20}
            object{ P_column2 scale <-1,1,1> translate x*30}
            object{ P_column2b scale <-1,1,1> translate x*40}
        
            object{ P_column1b translate x*10}
            object{ P_column1c translate x*20}
            object{ P_column1d scale <-1,1,1> translate x*30}
            object{ P_column1 scale <-1,1,1> translate x*40}
        
            object{ P_front1 translate x*10}
            object{ P_front1 translate x*20}
            object{ P_front1 translate x*30}
        
            union{object{P_window2_2}#if (Ref=1)object{P_glass2}#end object{ P_ledge2w } translate x*10}
            union{object{P_window2_3}#if (Ref=1)object{P_glass2}#end object{ P_ledge2w } translate x*20}
            union{object{P_window2_4}#if (Ref=1)object{P_glass2}#end object{ P_ledge2w } translate x*30}
            
            object{P_corner1 scale <-1,1,1> translate x*40}
            object{P_corner1a scale <-1,1,1> translate x*40}
            object{P_corner1b scale <-1,1,1> translate x*40}
            object{P_corner2 scale <-1,1,1> translate x*40}
            
            union{
                object{P_window1_right}
                object{P_curtain translate -z*1.1*12.649749/4}
                #if (Ref=1)  // excludes the glass panes from first pass
                    object{P_glass1}
                #end
                object{P_bricks2}
                translate x*30
            }
            object{ P_colside1 scale <-1,1,1> translate x*30}
            object{ P_colside1 translate x*10}
            object{ P_colside1 scale <-1,1,1> translate x*40}
            
            // these two lines are a hack because the model is only 1 floor high !
            object{ P_front1a translate z*15.5+y*0.5}
            object{ P_ledge2 translate z*15.5+y*0.5}

            rotate x*-90
            scale 4/12.649749
            scale <-1,1,1>
        }
        
        object{Building rotate y*180 translate -x*6.5}
        //object{Building rotate y*180 scale <1,1,-1> translate -z*11} // mirror building for reflection and radiosity
        
        #declare Sign=union{
            difference{torus{0.95,0.05 rotate x*-90}plane{z,0 inverse}}
            cylinder{-z*0.05,0,0.95}
            cylinder{0,z*0.05,1}
            texture{
                pigment{
                    image_map{
                        jpeg "parkingsign"
                    }
                }
                normal{wrinkles bump_size 0.03 scale 1/50}
                finish{ambient 0 diffuse 0.8 specular 0.8 roughness 1/10}
                translate <-0.5,-0.5,0>
                scale 2
            }                            
            
            translate -z*0.05
            scale <0.3,0.3,0.007/0.05>
        }
        //9.5
        object{Sign translate <-6.5,3.4,-0.1>}
        
        // various black boxes (block light)
        //box{<-100,-1,0>,<100,0.3,5> texture{pigment{Black}finish{ambient 0 diffuse 0}} translate z*1}
        //box{<-100,-1,0>,<100,20,5> texture{pigment{White*0.1}finish{ambient 0 diffuse 1}} translate z*2}
        //box{<-100,-1,-2>,<100,25,0> texture{pigment{White*0.1}finish{ambient 0 diffuse 1}} translate -z*13}
    #end
}

//---------------------------------------
// Mini Cooper
//---------------------------------------
#if (MiniOK=1)
    #debug "Mini\n"
    #declare V_WorldBoundMin = <-4.149374, -0.127721, -0.004524>;
    #declare V_WorldBoundMax = <5.696744, 23.635614, 10.148702>;
    #declare C_Paint= rgb <83,106,128>/255;
    #declare C_Paint_Roof = White*0.7;
    #declare C_Paint_Interior = rgb <229,206,157>/255;
    #declare angle_turn=-20; // rotation angle to turn the front wheels left (negative) or right (positive)
    #declare angle_move=30; // forward rotation angle for all the wheels (to be used in animation)
    #declare P_Paint=pigment{image_map{jpeg "car_paint2" interpolate 2}}
    #declare N_Paint=normal{
        average
        normal_map{
            [1 bump_map{jpeg "bd_door2d_bump" interpolate 2}]
            [1 bumps 0.2 scale 0.15]
        }
    }
    #declare T_paint = texture{
        image_pattern{jpeg "bd_door2d_bump" interpolate 2}
        texture_map{
            [0 pigment{P_Paint}normal{N_Paint} finish{ambient 0 diffuse 1 }]
//            [1 pigment{P_Paint} normal{N_Paint} finish{ambient 0 diffuse 0.5 specular 1 roughness 1/1000 reflection {0.1*Ref, 1*Ref falloff 3 exponent 2}}]
            [1 pigment{average pigment_map{[1 P_Paint][2 color C_Paint]}} normal{N_Paint} finish{ambient 0 diffuse 0.5 specular 1 roughness 1/1000 reflection {0.1*Ref, 0.8*Ref falloff 3 exponent 2}}]
        }   
        scale 1
    }
    #declare T_roof = texture{pigment{C_Paint_Roof}finish{ambient 0 diffuse 1 roughness 1/45 specular 1/3 reflection {0.1*Ref, 0.5*Ref}}}
    #declare T_bolts = texture{pigment{White*0.4} finish{ambient 0 diffuse 1 brilliance 1 metallic}}
    #declare T_bottom = texture{pigment{White*0.2} finish{ambient 0 diffuse 1}}
    #declare T_bulb = texture{pigment{Clear}}
    #declare T_chrome = texture{pigment{rgb <1,0.9,0.8>*0.7} finish{ambient 0 diffuse 1 brilliance 5 metallic specular 1 roughness 1/200 reflection{0.4*Ref, 0.99*Ref}}}
    #declare T_chrome = texture{
        pigment{rgb <1,0.9,0.8>*0.7} 
        finish{ambient 0 diffuse 0.7 brilliance 5 metallic specular 1 roughness 1/200 
            reflection{0.2*Ref, 0.6*Ref}
        }
    }
    #declare T_counterface = texture{pigment{Black}finish{reflection 0.2*Ref}}
    #declare T_glass = texture{pigment{rgbf <0.7,0.7,0.7,0.8>} finish{ambient 0 diffuse 0.1 specular 1 roughness 1/1000 reflection{0.1*Ref,0.7*Ref}}}
    #declare T_glass_hlight = texture{
        pigment{image_map{jpeg "headlight" transmit all 0.2 interpolate 2}} 
        normal{bump_map{jpeg "headlight_bmp" interpolate 2} bump_size -2} 
        finish{ambient 0 diffuse 1 specular 0.3 roughness 1/30 reflection {0.2*Ref,0.99*Ref}}
    }
    #declare T_glass_blight = texture{pigment{rgbf <0.8,0.5,0.1,0.3>} finish{ambient 0 diffuse 1 specular 0.3 roughness 1/30 reflection 0.2*Ref}}
    #declare T_interior = texture{pigment{C_Paint_Interior}finish{ambient 0 diffuse 1}}
    #declare T_logo_centre = texture{        
        pigment{image_map{jpeg "mini_emblem" interpolate 2}} 
        finish{ambient 0 diffuse 0.5 reflection 0.2*Ref}
    }
    #declare T_mirror = texture{pigment{Black}finish{reflection 0.99*Ref}}
    #declare T_plastic_black = texture{pigment{White*0.2} finish{ambient 0 diffuse 1 specular 1 roughness 1/15}}
    #declare T_plate = texture{
        pigment{
            image_map{jpeg "lplate" interpolate 2}
        } 
        finish{ambient 0 diffuse 0.6 specular 0.2 roughness 1/15}
    }
    #declare T_plate_rim = texture{T_plastic_black}
    #declare T_rubber = texture{pigment{bozo color_map{[0 White*0.01][1 White*0.3]}} finish{ambient 0 diffuse 1}}
    #declare T_seat = texture{T_interior}
    #declare T_tlbottom = texture{pigment{rgbf <0.8,0.1,0.1,0.3>} finish{ambient 0 diffuse 1 specular 0.3 roughness 1/30 reflection 0.2*Ref}}
    #declare T_tltop = texture{pigment{rgbf <0.8,0.5,0.1,0.3>} finish{ambient 0 diffuse 1 specular 0.3 roughness 1/30 reflection 0.2*Ref}}
    #declare T_wheel = texture{T_chrome}
    
    #declare T_austin_mat = texture{T_chrome}
    #declare T_bl_ch_mat = texture{T_chrome}
    #declare T_bl_gl_mat = texture{T_glass_blight}
    #declare T_bmp_fr_mat = texture{T_chrome}
    #declare T_bmp_rear_mat = texture{T_chrome}
    #declare T_body_mat = texture{T_paint}
    #declare T_body2_mat = texture{T_paint scale 2}
    #declare T_bolts_fr_mat = texture{T_bolts}
    #declare T_bolts_rear_mat = texture{T_bolts}
    #declare T_bottom_fr_mat = texture{T_bottom}
    #declare T_bottom_mat = texture{T_bottom}
    #declare T_bs_bk_mat = texture{T_seat}
    #declare T_bs_bt_mat = texture{T_seat}
    #declare T_bt_ch_mat = texture{T_chrome}
    #declare T_cooper_mat = texture{T_chrome}
    #declare T_cterbase_mat = texture{T_plastic_black}
    #declare T_ctrface_mat = texture{T_counterface}
    #declare T_ctrrim_mat = texture{T_chrome}
    #declare T_dashboard_mat = texture{T_interior}
    #declare T_DEFAULT = texture{T_interior}
    #declare T_dw_rim_mat = texture{T_plastic_black}
    #declare T_dw2_mat = texture{T_chrome}
    #declare T_dw3_mat = texture{T_chrome}
    #declare T_dwshaft_mat = texture{T_plastic_black}
    #declare T_exhaust_mat = texture{T_chrome}
    #declare T_front_mat = texture{T_paint scale 2}
    #declare T_fs_bk_mat = texture{T_seat}
    #declare T_fs_bt_mat = texture{T_seat}
    #declare T_gascap_mat = texture{T_chrome}
    #declare T_gl_fr_mat = texture{T_glass}
    #declare T_gl_rear_mat = texture{T_glass}
    #declare T_gl_side1_mat = texture{T_glass}
    #declare T_gl_side2_mat = texture{T_glass}
    #declare T_gr_bk_mat = texture{T_paint}
    #declare T_gr_fr1_mat = texture{T_paint}
    #declare T_gr_fr2_mat = texture{T_paint}
    #declare T_handle_mat = texture{T_chrome}
    #declare T_hinge1_mat = texture{T_paint}
    #declare T_hinge2_mat = texture{T_paint}
    #declare T_hinge3_mat = texture{T_paint}
    #declare T_hinge4_mat = texture{T_paint}
    #declare T_hl_bulb_mat = texture{T_mirror}
    #declare T_hl_ch_mat = texture{T_chrome}
    #declare T_hl_gl_mat = texture{T_glass_hlight}
    #declare T_hl_mir_mat = texture{T_mirror}
    #declare T_hood_mat = texture{T_paint scale 2}
    #declare T_hoodtop_mat = texture{T_paint}
    #declare T_i_body1_mat = texture{T_interior}
    #declare T_i_body2_mat = texture{T_interior}
    #declare T_i_fl_rear_mat = texture{T_bottom}
    #declare T_i_floor_fr_mat = texture{T_bottom}
    #declare T_i_floor_mat = texture{T_interior}
    #declare T_i_floor2_mat = texture{T_interior}
    #declare T_i_handle1_mat = texture{T_chrome}
    #declare T_i_handle2_mat = texture{T_chrome}
    #declare T_i_handle3_mat = texture{T_chrome}
    #declare T_i_rear_mat = texture{T_interior}
    #declare T_i_roof_mat = texture{T_interior}
    #declare T_i_rv_mat = texture{T_chrome}
    #declare T_i_rvmir_mat = texture{T_mirror}
    #declare T_lgcentre_mat = texture{T_logo_centre}
    #declare T_lgwings_mat = texture{T_chrome}
    #declare T_lwip1_mat = texture{T_chrome}
    #declare T_lwip2_mat = texture{T_chrome}
    #declare T_pedal1_mat = texture{T_plastic_black}
    #declare T_pedal2_mat = texture{T_plastic_black}
    #declare T_pl_fr_mat = texture{T_plate}
    #declare T_pl_frrim_mat = texture{T_plastic_black}
    #declare T_pl_rear_mat = texture{T_plate}
    #declare T_pl_rrim_mat = texture{T_plastic_black}
    #declare T_radiator_mat = texture{T_chrome}
    #declare T_rear_hd_mat = texture{T_chrome}
    #declare T_rear_hinge_mat = texture{T_paint}
    #declare T_rear_mat = texture{T_paint}
    #declare T_roof_mat = texture{T_roof}
    #declare T_roof_trim_mat = texture{T_paint}
    #declare T_rv_side_mat = texture{T_chrome}
    #declare T_rv_sidemir_mat = texture{T_mirror}
    #declare T_rwip1_mat = texture{T_chrome}
    #declare T_rwip2_mat = texture{T_chrome}
    #declare T_sprinkler_mat = texture{T_plastic_black}
    #declare T_switch_mat = texture{T_plastic_black}
    #declare T_throthead_mat = texture{T_plastic_black}
    #declare T_throtrub_mat = texture{T_plastic_black}
    #declare T_throtshaft_mat = texture{T_chrome}
    #declare T_tl_ch_mat = texture{T_chrome}
    #declare T_tl_glbt_mat = texture{T_tlbottom}
    #declare T_tl_gltop_mat = texture{T_tltop}
    #declare T_tyre_fr_mat = texture{T_rubber}
    #declare T_tyre_rear_mat = texture{T_rubber}
    #declare T_wd_ch1_mat = texture{T_chrome}
    #declare T_wd_ch2_mat = texture{T_chrome}
    #declare T_wd_ch3_mat = texture{T_chrome}
    #declare T_wd_chbk_mat = texture{T_chrome}
    #declare T_wd_chfr_mat = texture{T_chrome}
    #declare T_wh_fr1_mat = texture{T_chrome}
    #declare T_wh_fr2_mat = texture{T_chrome}
    #declare T_wh_fr3_mat = texture{T_chrome}
    #declare T_wh_rear1_mat = texture{T_chrome}
    #declare T_wh_rear2_mat = texture{T_chrome}
    #declare T_wh_rear3_mat = texture{T_chrome}
    
    #include "mini_mesh_o.inc"
    #declare FrontWheel=union{
        object{ P_tyre_fr }
        object{ P_bolts_fr }
        object{ P_wh_fr3 }
        object{ P_wh_fr2 }
        object{ P_wh_fr1 }
    }
    #declare RearWheel=union{
        object{ P_wh_rear1 }
        object{ P_wh_rear2 }
        object{ P_wh_rear3 }
        object{ P_bolts_rear }
        object{ P_tyre_rear }
    }
       
    #declare Halfcar=union{
        
        union{
            object{ P_body }
            object{ P_hood }
            object{ P_rear }
            object{ P_body2 }
            object{ P_front }
            object{ P_hoodtop }
            object{ P_gr_bk }
            object{ P_gr_fr1 }
            object{ P_gr_fr2 }
            object{ P_rear_hinge }
            object{ P_hinge1 }
            object{ P_roof_trim }
            object{ P_hinge2 }
            object{ P_hinge4 }
            object{ P_hinge3 }
//            interior{ior 10}
        }
        object{ P_i_fl_rear }
        object{ P_i_floor }
        object{ P_i_floor_fr }
        object{ P_dashboard }
        object{ P_roof }
        object{ P_sprinkler }
        object{ P_radiator }
        object{ P_handle }
        object{ P_wd_ch1 }
        object{ P_wd_ch2 }
        object{ P_wd_ch3 }
        object{ P_wd_chfr }
        object{ P_wd_chbk }
        object{ P_bt_ch }
        object{ P_hl_gl }
        object{ P_hl_ch }
        object{ P_hl_bulb }
        object{ P_hl_mir }
        object{ P_bmp_fr }
        object{ P_bottom }
        object{ P_bottom_fr }
        object{ P_gascap }
        object{ P_tl_ch }
        object{ P_bmp_rear }
        object{ P_bl_gl }
        object{ P_bl_ch }
        
        #if (Ref=1) // excludes the glass panes from first pass
            object{ P_gl_fr }
            object{ P_gl_rear }
            object{ P_gl_side1 }
            object{ P_gl_side2 }
        #end
        
        object{ P_rv_side }
        object{ P_rv_sidemir }
        object{ P_rear_hd }
        object{ P_i_roof }
        object{ P_bs_bt }
        object{ P_fs_bk }
        object{ P_bs_bk }
        object{ P_i_rear }
        object{ P_i_body1 }
        object{ P_i_body2 }
        object{ P_fs_bt }
        object{ P_i_handle1 }
        object{ P_i_handle3 }
        object{ P_pedal2 }
        object{ P_pedal1 }
        object{ P_i_handle2 }
        object{ P_tl_gltop }
        object{ P_tl_glbt }
    }
    
    
    #declare MiniCooper=union{
        object{Halfcar}
        object{Halfcar scale <-1,1,1>}
        
        object{ P_exhaust }
        object{ P_switch }
        object{ P_i_rv }
        object{ P_i_rvmir }
        object{ P_dw_rim }
        object{ P_dw2 }
        object{ P_dw3 }
        object{ P_dwshaft }
        object{ P_throthead }
        object{ P_throtshaft }
        object{ P_throtrub }
        object{ P_lwip1 }
        object{ P_lwip2 }
        object{ P_rwip1 }
        object{ P_rwip2 }
        object{ P_cterbase }
        object{ P_ctrrim }
        object{ P_ctrface }
        object{ P_lgwings }
        object{ P_lgcentre }
        object{ P_pl_frrim }
        object{ P_pl_fr }
        object{ P_austin }
        object{ P_cooper }
        object{ P_pl_rrim }
        object{ P_pl_rear }
        object{ FrontWheel translate <-4.8,-4,-2> rotate x*angle_move rotate z*angle_turn translate <4.8,4,2>}
        object{ FrontWheel translate <-4.8,-4,-2> rotate x*(10+angle_move) rotate -z*angle_turn translate <4.8,4,2> scale <-1,1,1>}
        object{ RearWheel translate <-4.8,-19.5,-2> rotate x*(42+angle_move) translate <4.8,19.5,2>} 
        object{ RearWheel translate <-4.8,-19.5,-2> rotate x*(-10+angle_move) translate <4.8,19.5,2> scale <-1,1,1>} 

        rotate x*-90
        translate z*V_WorldBoundMax.y/2
        
        scale 1.4/10.148702
        scale <-1,1,1>
        rotate y*180
        
    }
    object{MiniCooper rotate y*-90 translate car_location}

#end
