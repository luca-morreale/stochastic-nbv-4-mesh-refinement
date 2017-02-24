// *** PAPERS ON THE STREET ***
#local r_paper=seed(222);
#declare papers_on_street=
union{
// in the street
#declare num_papers=12;
#declare cnt_papers=1;
#while (cnt_papers<=num_papers)
 #declare rnd_color=rgb <.1*rand(r_paper),.1*rand(r_paper),.1*rand(r_paper)>;
 object{wrinkled_paper(rand(r_paper),8)
   texture{
    pigment{Gray80+rnd_color}
    normal{wrinkles scale 2}
    finish{Dull}
   }
   texture{t_dirty}
   translate <-450+900*rand(r_paper),3.5,190+105*rand(r_paper)>
 }
 #declare cnt_papers=cnt_papers+1;
#end
}
#declare papers_on_sidew=
union{
// over the sidewalk
#declare num_papers=8;
#declare cnt_papers=1;
#while (cnt_papers<=num_papers)
 #declare rnd_color=rgb <.1*rand(r_paper),.1*rand(r_paper),.1*rand(r_paper)>;
 object{wrinkled_paper(rand(r_paper),8)
   texture{
    pigment{Gray80+rnd_color}
    normal{wrinkles scale 2}
    finish{Dull}
   }
   texture{t_dirty}
   translate <-450+900*rand(r_paper),15,300+290*rand(r_paper)>
 }
 #declare cnt_papers=cnt_papers+1;
#end
}
#declare papers_on_tree=
union{
// in the tree terrain
#local r_paper=seed(6533);
#declare num_papers=11;
#declare cnt_papers=1;
#while (cnt_papers<=num_papers)
 #declare rnd_color=rgb <.1*rand(r_paper),.1*rand(r_paper),.1*rand(r_paper)>;
 object{wrinkled_paper(rand(r_paper),8)
   texture{
    pigment{Gray80+rnd_color}
    normal{wrinkles scale 2}
    finish{Dull}
   }
   texture{t_dirty}
   translate <-60+120*rand(r_paper),0,-60+120*rand(r_paper)>
 }
 #declare cnt_papers=cnt_papers+1;
#end
}