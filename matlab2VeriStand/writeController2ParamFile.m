function writeController2ParamFile(controller, filename)

% controller is a discrete-time state space system

fid = fopen(filename,'w');

delim = ' '; % space

Ac = controller.A'; % transpose get it to row major form
Ac = Ac(:);
str = matlab2VSparam(Ac,delim);
fprintf(fid,'%s\n',str);

Bc = controller.B';
Bc = Bc(:);
str = matlab2VSparam(Bc,delim);
fprintf(fid,'%s\n',str);

Cc = controller.C';
Cc = Cc(:);
str = matlab2VSparam(Cc,delim);
fprintf(fid,'%s\n',str);

Dc = controller.D';
Dc = Dc(:);
str = matlab2VSparam(Dc,delim);
fprintf(fid,'%s\n',str);

fclose(fid);
