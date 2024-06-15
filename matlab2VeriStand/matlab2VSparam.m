function str = matlab2VSparam(var)

% input: 
% var = nx1 vector
% output
% str = string: 'varname=[var(1) delim var(2) delim ...var(n)]'

str = inputname(1);
str = str + "=[";
for n = 1:length(var)-1
    str = str + num2str(var(n)) + " ";
end
str = str + num2str(var(end)) + "]";
