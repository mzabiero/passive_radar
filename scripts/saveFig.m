fig = gcf;
set(fig, "Renderer", "painters");
fold = '/home/mzabiero/studia/inzynierka/latex/img/03_ch3/simulation/scenario3/';
name = 'drone5_filt.png';
fullName =  strcat(fold, name);
exportgraphics(fig,fullName, 'Resolution', 600);