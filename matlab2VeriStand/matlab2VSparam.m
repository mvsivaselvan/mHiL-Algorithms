function str = matlab2VSparam(var)

% input: 
% var = nx1 vector
% output
% str = string: 'varname=[var(1);var(2);...var(n)]'

str = inputname(1);
str = strcat(str,'=[');
for n = 1:length(var)-1
    str = strcat(str,num2str(var(n)),';');
end
str = strcat(str,num2str(var(end)),']');
