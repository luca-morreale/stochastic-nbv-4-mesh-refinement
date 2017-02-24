// *** ORANGE TREE ***

// *** tree parameters ***
#declare base_radius           =6; // trunk base radius
#declare top_radius            =1;  // trunk top radius (keep it low)
#declare sections              =40; // total trunk sections
#declare start_section_lenght  =8; // first section lenght
#declare end_section_lenght    =6;  // last section lenght
#declare branch_start_section  =15; // trunk section for first branch
#declare branch_end_section    =40; // trunk section for last, vertical branch
#declare startbranch_sections  =15; // number of sections for 1st branch
#declare startsbranch_sections =10; //        "            "   " sub-branch
#declare startlbranch_sections =10; //        "            "   " last-branch
#declare branch_v_min_angle    =30; // minimum vertical angle for branches
#declare branch_v_max_angle    =40; //    "       "       "    "  branches
#declare branch_h_rotation     =60; // succesive rotation angle for branches (horizontal)
#declare sbranch_v_min_angle   =10; // idem for sub-branches
#declare sbranch_v_max_angle   =20;
#declare sbranch_h_rotation    =60;
#declare fbranch_v_min_angle   =20; // idem for last-branches
#declare fbranch_v_max_angle   =30;
#declare fbranch_h_rotation    =60;
#declare leaves_rotation       =60; // succ. horizontal rotation for leaves
#declare branchs_per_section   =2;  // number of branches per section
#declare branch_percentage     =0.6; // percentage of branches (1 means all sections have branches)
#declare sbranch_percentage    =0.6; // idem for sub-branches
#declare lbranch_percentage    =0.6; // idem for last-branches
#declare leaves_percentage     =0.6; // idem for leaves
#declare verticality           =0.9; // trunk/branch verticallity percentage
#declare curves                =0.2; //      "       curves percentage
#declare number_of_roots       =3;
#declare bark_texture          =texture{t_bark}
#declare i_leaf                ="i_leaves.inc"
#declare l_switch              =on; // leaves on/off
#declare f_switch              =on; // fruits on/off
#declare random_seed           =114;

#declare orange_tree=
#include "i_tree.inc"
