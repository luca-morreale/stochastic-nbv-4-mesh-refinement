/*

  Tree Include File, by Jaime Vives Piqueres, (?)1999.

  It was originally a macro, but at some moment POV-Ray
  said "there are too many macro parameters"! :(

  Needs "treem.inc" for the branches macros.

*/

/*

  Note: the following variables must be defined prior to call this include
  file. Also, you need first a call to "treemacr.inc" to define the macros
  for the branch levels (see test.pov for an example of use).

  base_radius            // n
  top_radius             // n
  sections               // n
  start_section_lenght   // n
  end_section_lenght     // n
  branch_start_section   // n
  branch_end_section     // n
  startbranch_sections   // n
  startsbranch_sections  // n
  startlbranch_sections  // n
  branch_v_min_angle     // a (0-360)
  branch_v_max_angle     // a (0-360)
  branch_h_rotation      // a (0-360)
  sbranch_v_min_angle    // a (0-360)
  sbranch_v_max_angle    // a (0-360)
  sbranch_h_rotation     // a (0-360)
  fbranch_v_min_angle    // a (0-360)
  fbranch_v_max_angle    // a (0-360)
  fbranch_h_rotation     // a (0-360)
  branchs_per_section    // n
  branch_percentage      // % (0-1)
  sbranch_percentage     // % (0-1)
  lbranch_percentage     // % (0-1)
  leaves_percentage      // % (0-1)
  verticality            // % (0-1)
  curves                 // % (0-1)
  number_of_roots        // n
  l_switch               // on/off
  bark_texture           // texture
  i_leaf                 // include file name
  random_seed            // n

*/

 // prepare work variables
 #local radius_decrement=(base_radius-top_radius)/sections;
 #local section_decrement=(start_section_lenght-end_section_lenght)/sections;
 #local rad_ant=base_radius;
 #local x_ant=0;
 #local y_ant=0;
 #local z_ant=0;
 #local this_section_lenght=start_section_lenght;
 #local r_tree=seed(random_seed);
 #local cnt=0;
 #local sign_x=1;
 #local sign_z=1;
 #local x_inc=0;
 #local z_inc=0;
 #local x_act=0;
 #local z_act=0;

 // that's the trunk loop, which calls the branch macro
 union{
 #while (cnt<(branch_end_section))
  // values for this section
  #local rad_act=rad_ant-radius_decrement;
  #if (rand(r_tree)>verticality)
    #if (rand(r_tree)>.5)
      #local sign_x=sign_x*(-1);
    #else
      #local sign_z=sign_z*(-1);
    #end
  #end
  #if (rand(r_tree)<curves)
   #if (rand(r_tree)>.5)
    #local x_inc=this_section_lenght*.5*rand(r_tree)*(1-(cnt/sections));
   #else
    #local z_inc=this_section_lenght*.5*rand(r_tree)*(1-(cnt/sections));
   #end
  #end
  #local x_act=x_ant+x_inc*sign_x;
  #local z_act=z_ant+z_inc*sign_z;
  #local y_act=y_ant+this_section_lenght-abs(x_inc)-abs(z_inc);
  // this section cone and sphere
  cone{
   <x_ant,y_ant,z_ant>,rad_ant,
   <x_act,y_act,z_act>,rad_act
   texture{bark_texture}
  }
  sphere{<x_ant,y_ant,z_ant>,rad_ant
   texture{bark_texture}
  }
  // branches on this section
  #if ((cnt>(branch_start_section-1)) & (cnt<(sections-1)))
   #local cnt_branchs=0;
   #while (cnt_branchs<branchs_per_section)
    #if (rand(r_tree)<branch_percentage)
     object{
      branch(
       rad_act*.75,
       top_radius*.75,
       (startbranch_sections-cnt*.5)*.75+(startbranch_sections-cnt*.5)*.25*rand(r_tree),
       this_section_lenght,
       end_section_lenght,
       branch_v_min_angle+(branch_v_max_angle-branch_v_min_angle)*rand(r_tree),
       branch_h_rotation*cnt
      )
      rotate (360*cnt_branchs/branchs_per_section)*y
      translate <x_act,y_act,z_act>
     }
    #end
    #local cnt_branchs=cnt_branchs+1;
   #end
  #end
  // update values for next section
  #local x_ant=x_act;
  #local y_ant=y_act;
  #local z_ant=z_act;
  #local rad_ant=rad_act;
  #local this_section_lenght=this_section_lenght-section_decrement;
  #local cnt=cnt+1;
 #end
 // final branch at trunk end
 #if (branch_end_section<sections)
  object{
   branch(
    rad_act,
    top_radius,
    sections-branch_end_section,
    this_section_lenght,
    end_section_lenght,
    0,
    0
   )
   translate <x_act,y_act,z_act>
  }
 #end
 // roots, done with lastbranch() macro
 #local cnt=0;
 #declare l_switch=off;
 #while (cnt<number_of_roots)
  object{
   lastbranch(
    (base_radius/number_of_roots)+(base_radius/number_of_roots)*rand(r_tree),
    top_radius,
    sections*.25+sections*.25*rand(r_tree),
    start_section_lenght,
    end_section_lenght,
    0,
    0
   )
   rotate -(150+20*rand(r_tree))*x
   translate -(base_radius*.25+base_radius*.25*rand(r_tree))*z
   rotate (360*cnt/number_of_roots)*y
   translate (start_section_lenght*.5+start_section_lenght*rand(r_tree))*y
  }
  #local cnt=cnt+1;
 #end
 }

