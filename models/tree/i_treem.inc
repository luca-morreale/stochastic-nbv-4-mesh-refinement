/*

  Branch macros for use with "tree.inc", Jaime Vives Piqueres, (?)1999.
  They work as the trunk on tree.inc, branch() calling subbranch(), and
  subbranch() calling lastbranch(), wich finally includes the leaf file.

*/

// *** last branch macro ***
#macro lastbranch(
  base_radius,           // n
  top_radius,            // n
  sections,              // n
  start_section_lenght,  // n
  end_section_lenght,    // n
  v_angle,               // 0-360
  h_rotation             // 0-360
 )

 #local radius_decrement=(base_radius-top_radius)/sections;
 #local section_decrement=(start_section_lenght-end_section_lenght)/sections;
 #local rad_ant=base_radius;
 #local x_ant=0;
 #local y_ant=0;
 #local z_ant=0;
 #local this_section_lenght=start_section_lenght;
 #local cnt=0;
 #local sign_x=1;
 #local sign_z=1;
 #local x_inc=0;
 #local z_inc=0;
 #local x_act=0;
 #local z_act=0;

 union{
 #while (cnt<sections)
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
     #local x_inc=this_section_lenght*.5*rand(r_tree); //*(1-(cnt/sections));
    #else
     #local z_inc=this_section_lenght*.5*rand(r_tree); //*(1-(cnt/sections));
    #end
  #end
  #local x_act=x_ant+x_inc*sign_x;
  #local z_act=z_ant+z_inc*sign_z;
  #local y_act=y_ant+this_section_lenght-abs(x_inc)-abs(z_inc);
  cone{
   <x_ant,y_ant,z_ant>,rad_ant,
   <x_act,y_act,z_act>,rad_act
   texture{bark_texture}
  }
  sphere{<x_ant,y_ant,z_ant>,rad_ant
   texture{bark_texture}
  }
  #if (l_switch)
   #if (rand(r_tree)<leaves_percentage)
    object{
     #include i_leaf
     rotate leaves_rotation*cnt*y
     translate <x_act,y_act,z_act>
    }
    #if (f_switch)
     #if (rand(r_tree)>0.9)
      object{
       #include "i_fruit.inc"
       translate <x_act,y_act,z_act-8>
      }
     #end
    #end
   #end
  #end
  #local x_ant=x_act;
  #local y_ant=y_act;
  #local z_ant=z_act;
  #local rad_ant=rad_act;
  #local this_section_lenght=this_section_lenght-section_decrement;
  #local cnt=cnt+1;
 #end
  rotate -v_angle*x
  rotate h_rotation*y
 }

#end

// *** second branch macro ***
#macro subbranch(
  base_radius,           // n
  top_radius,            // n
  sections,              // n
  start_section_lenght,  // n
  end_section_lenght,    // n
  v_angle,               // 0-360
  h_rotation             // 0-360
 )

 #local radius_decrement=(base_radius-top_radius)/sections;
 #local section_decrement=(start_section_lenght-end_section_lenght)/sections;
 #local rad_ant=base_radius;
 #local x_ant=0;
 #local y_ant=0;
 #local z_ant=0;
 #local this_section_lenght=start_section_lenght;
 #local cnt=0;
 #local sign_x=1;
 #local sign_z=1;
 #local x_inc=0;
 #local z_inc=0;
 #local x_act=0;
 #local z_act=0;

 union{
 #while (cnt<sections)
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
    #local x_inc=this_section_lenght*.5*rand(r_tree); //*(1-(cnt/sections));
   #else
    #local z_inc=this_section_lenght*.5*rand(r_tree); //*(1-(cnt/sections));
   #end
  #end
  #local x_act=x_ant+x_inc*sign_x;
  #local z_act=z_ant+z_inc*sign_z;
  #local y_act=y_ant+this_section_lenght-abs(x_inc)-abs(z_inc);
  cone{
   <x_ant,y_ant,z_ant>,rad_ant,
   <x_act,y_act,z_act>,rad_act
   texture{bark_texture}
  }
  sphere{<x_ant,y_ant,z_ant>,rad_ant
   texture{bark_texture}
  }
  #if ((cnt>0) & (cnt<sections))
   #if (rand(r_tree)<lbranch_percentage)
    object{
     lastbranch(
      rad_ant,
      top_radius,
      (startlbranch_sections-cnt*.5)*.75+(startlbranch_sections-cnt*.5)*.25*rand(r_tree),
      this_section_lenght,
      end_section_lenght,
      fbranch_v_min_angle+(fbranch_v_max_angle-fbranch_v_min_angle)*rand(r_tree),
      fbranch_h_rotation*cnt
     )
     translate <x_act,y_act,z_act>
    }
   #end
  #end
  #local x_ant=x_act;
  #local y_ant=y_act;
  #local z_ant=z_act;
  #local rad_ant=rad_act;
  #local this_section_lenght=this_section_lenght-section_decrement;
  #local cnt=cnt+1;
 #end
  rotate -v_angle*x
  rotate h_rotation*y
 }

#end

// *** first branch macro ***
#macro branch(
  base_radius,           // n
  top_radius,            // n
  sections,              // n
  start_section_lenght,  // n
  end_section_lenght,    // n
  v_angle,               // 0-360
  h_rotation             // 0-360
 )

 #local radius_decrement=(base_radius-top_radius)/sections;
 #local section_decrement=(start_section_lenght-end_section_lenght)/sections;
 #local rad_ant=base_radius;
 #local x_ant=0;
 #local y_ant=0;
 #local z_ant=0;
 #local this_section_lenght=start_section_lenght;
 #local cnt=0;
 #local sign_x=1;
 #local sign_z=1;
 #local x_inc=0;
 #local z_inc=0;
 #local x_act=0;
 #local z_act=0;

 union{
 #while (cnt<sections)
  #local rad_act=rad_ant-radius_decrement;
  #if (rand(r_tree)>verticality)
    #local sign_x=sign_x*(-1);
    #local sign_z=sign_z*(-1);
  #end
  #if (rand(r_tree)<curves)
   #if (rand(r_tree)>.5)
    #local x_inc=this_section_lenght*.5*rand(r_tree); //*(1-(cnt/sections));
   #else
    #local z_inc=this_section_lenght*.5*rand(r_tree); //*(1-(cnt/sections));
   #end
  #end
  #local x_act=x_ant+x_inc*sign_x;
  #local z_act=z_ant+z_inc*sign_z;
  #local y_act=y_ant+this_section_lenght-abs(x_inc)-abs(z_inc);
  cone{
   <x_ant,y_ant,z_ant>,rad_ant,
   <x_act,y_act,z_act>,rad_act
   texture{bark_texture}
  }
  sphere{<x_ant,y_ant,z_ant>,rad_ant
   texture{bark_texture}
  }
  #if ((cnt>0) & (cnt<sections))
   #if (rand(r_tree)<sbranch_percentage)
    object{
     subbranch(
      rad_act,
      top_radius,
      (startsbranch_sections-cnt*.5)*.75+(startsbranch_sections-cnt*.5)*.25*rand(r_tree),
      this_section_lenght,
      end_section_lenght,
      sbranch_v_min_angle+(sbranch_v_max_angle-sbranch_v_min_angle)*rand(r_tree),
      sbranch_h_rotation*cnt
     )
     translate <x_act,y_act,z_act>
    }
   #end
  #end
  #local x_ant=x_act;
  #local y_ant=y_act;
  #local z_ant=z_act;
  #local rad_ant=rad_act;
  #local this_section_lenght=this_section_lenght-section_decrement;
  #local cnt=cnt+1;
 #end
  rotate v_angle*x
  rotate h_rotation*y
 }

#end
