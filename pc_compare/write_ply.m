function [] = write_ply(fname, P, C)
    % Written by Chenxi cxliu@ucla.edu
    % Input: fname: output file name, e.g. 'data.ply'
    %        P: 3*m matrix with the rows indicating X, Y, Z
    %        C: 3*m matrix with the rows indicating R, G, B

    num = size(P, 2);
    header = 'ply\n';
    header = [header, 'format ascii 1.0\n'];
    header = [header, 'comment written by Chenxi\n'];
    header = [header, 'element vertex ', num2str(num), '\n'];
    header = [header, 'property float32 x\n'];
    header = [header, 'property float32 y\n'];
    header = [header, 'property float32 z\n'];
    header = [header, 'property uchar red\n'];
    header = [header, 'property uchar green\n'];
    header = [header, 'property uchar blue\n'];
    header = [header, 'end_header\n'];

    data = [P', double(C')];

    fid = fopen(fname, 'w');
    fprintf(fid, header);
    dlmwrite(fname, data, '-append', 'delimiter', '\t', 'precision', 3);
    fclose(fid);

end