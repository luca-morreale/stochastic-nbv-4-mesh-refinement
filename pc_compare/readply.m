function [pc] = readply(filename)

idx = 1;
fid = fopen(filename, 'r');
while isempty(strfind(fgets(fid), 'end_header'))
    idx = idx + 1;
end
fclose(fid);

data = textread(filename, '%s','delimiter', '\n');
data = data(idx+1:length(data),1);
data = (cellfun(@(x) strread(x,'%s','delimiter',' '), data, 'UniformOutput', false));

if isempty(data{length(data)})
    data(length(data))=[];
end

pc = str2double([data{:}].');
pc = pc(:,1:6);

end