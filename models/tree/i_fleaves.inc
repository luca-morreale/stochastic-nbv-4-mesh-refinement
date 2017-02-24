// *** fallen leaves in the tree terrain ***
#local r_fleaves=seed(414);
#declare num_fleaves=20;
#declare cnt_fleaves=1;
#declare fleaves=
union{
#while (cnt_fleaves<=num_fleaves)
 #declare rnd_color=rgb <.1*rand(r_fleaves),.1*rand(r_fleaves),.1*rand(r_fleaves)>;
 sphere{0,1
  texture{t_leaves
   normal{wrinkles}
   translate 10*rand(r_fleaves)*z}
  scale <.4,.01,1>
  scale 4
  rotate 360*rand(r_fleaves)*y
  translate <-60+120*rand(r_fleaves),0,-60+120*rand(r_fleaves)>
  translate <-25,9,375>
 }
 #declare cnt_fleaves=cnt_fleaves+1;
#end
}