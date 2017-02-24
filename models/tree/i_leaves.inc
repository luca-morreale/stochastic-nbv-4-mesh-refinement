// *** leaves object for tree.inc ***
#local cnt=1;
#local r_leaf=seed(rand(r_tree));
#local leaf_count=6+3*rand(r_leaf);
#local tmp=0;
#local sc=0;
union{
 #while (cnt<=leaf_count)
  #local tmp=this_section_lenght*.5+this_section_lenght*.5*rand(r_leaf);
  #local sc=4+rand(r_leaf);
  union{
   sphere{0,1
    texture{t_leaves translate 10*rand(r_leaf)*z}
    scale <.4,1,.01>
    scale sc
    rotate 45*x
    translate <0,-sc*.5,-tmp>
   }
   cylinder{<0,0,0>,<0,0,-tmp>,.5 texture{t_bark}}
   rotate 90*rand(r_leaf)*x
   rotate 360*(cnt/leaf_count)*y
  }
  #local cnt=cnt+1;
 #end
}
