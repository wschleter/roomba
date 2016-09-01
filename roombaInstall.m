% installation program for EF 230 Roomba Project
clc;
prompt = {
    'This program will download the EF 230 Roomba files'
    ['Current folder: ' cd ] 
    'Do you want to continue? '
    };
beep;
yn = questdlg(prompt, ...
	'Roomba Install/Update', ...
	'Yes','No','Yes');

if length(yn>0) && upper(yn(1))=='Y'
    files = {'roomba.m','roombaSim.m','roombaSimGUI.m','roombaSimGUI.fig','roombaInstall.m'};
    gh = 'https://raw.githubusercontent.com/wschleter/roomba/master/';
    for f=files
        f=f{1};
        disp(['Downloading ' f]);
        urlwrite([gh f],f);
    end
end
