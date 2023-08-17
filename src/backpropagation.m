function varargout = backpropagation(varargin)
% BACKPROPAGATION MATLAB code for backpropagation.fig
%      BACKPROPAGATION, by itself, creates a new BACKPROPAGATION or raises the existing
%      singleton*.
%
%      H = BACKPROPAGATION returns the handle to a new BACKPROPAGATION or the handle to
%      the existing singleton*.
%
%      BACKPROPAGATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BACKPROPAGATION.M with the given input arguments.
%
%      BACKPROPAGATION('Property','Value',...) creates a new BACKPROPAGATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before backpropagation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to backpropagation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help backpropagation

% Last Modified by GUIDE v2.5 04-Apr-2022 09:24:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @backpropagation_OpeningFcn, ...
                   'gui_OutputFcn',  @backpropagation_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before backpropagation is made visible.
function backpropagation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to backpropagation (see VARARGIN)

% Choose default command line output for backpropagation
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes backpropagation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = backpropagation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_insert.
function pb_insert_Callback(hObject, eventdata, handles)
% hObject    handle to pb_insert (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[namafile, formatfile] = uigetfile({'*.jpg'}, 'membuka gambar');
image = imread([formatfile, namafile]);

M = imread(namafile);
[baris,kolom] = size(M);
while (kolom>700)
    M = imresize(M,0.5);
    [baris,kolom] = size(M);
end
handles.M=M;
handles.namafile = namafile;
axes(handles.axes1);
imshow(M);
guidata(hObject, handles);

% --- Executes on button press in pb_matrix.
function pb_matrix_Callback(hObject, eventdata, handles)
% hObject    handle to pb_matrix (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

img = handles.img;
namafile = handles.namafile;
threshold = graythresh (img);
img = ~imbinarize(img, threshold);
img = im2double(img);
set(handles.uitable1, 'Data', img);

% --- Executes on button press in pb_train.
function pb_train_Callback(hObject, eventdata, handles)
% hObject    handle to pb_train (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
M = handles.M;
threshold=graythresh(M);
M=~imbinarize(M,threshold);
M=im2double(M);
M=bwareaopen(M,15);

image_data = imresize(M,[10,10]);
image1=image_data(:,:,1);
image1(image1==0)=-1;
x=reshape(image1,[1,100]);
namafile = handles.namafile;
%Pembentukan Pola Huruf untuk Pengenalan 
D1 = imread('D1.jpg');
threshold1=graythresh(D1);
P1=~imbinarize(D1,threshold1);
P1=im2double(P1);

E1 = imread('E1.jpg');
threshold2=graythresh(E1);
P2=~imbinarize(E1,threshold2);
P2=im2double(P2);

F1 = imread('F1.jpg');
threshold3=graythresh(F1);
P3=~imbinarize(F1,threshold3);
P3=im2double(P3);

D2 = imread('D2.jpg');
threshold4=graythresh(D2);
P4=~imbinarize(D2,threshold4);
P4=im2double(P4);

E2 = imread('E2.jpg');
threshold5=graythresh(E2);
P5=~imbinarize(E2,threshold5);
P5=im2double(P5);

F2 = imread('F2.jpg');
threshold6=graythresh(F2);
P6=~imbinarize(F2,threshold6);
P6=im2double(P6);

%target
t = ones(1,10);
if image == 'D1.jpg'
    t(2:10)=0;
elseif image=='E1.jpg'
    t(1)=0;
    t(3:10)=0;
elseif image=='F1.jpg'
    t(1:2)=0;
    t(4:10)=0;
elseif image == 'D2.jpg'
    t(2:10)=0;
elseif image=='E2.jpg'
    t(1)=0;
    t(3:10)=0;
elseif image=='F2.jpg'
    t(1:2)=0;
    t(4:10)=0;
end

% Algoritma BP
% step 0
n = length(x);
m = length(t);
p = 5;
z = zeros(p,1);
v = xlsread('Hasil_latih.xlsx','sheet3','A1:E100');
w = xlsread('Hasil_latih.xlsx','sheet1','A1:J5');
v_0 = xlsread('Hasil_latih.xlsx','sheet4','A1:A5');
w_0 = xlsread('Hasil_latih.xlsx','sheet2','A1:A10');
theta = 0.05;
alpha = 1;
test = 0;
% step 1
% while test ~= 1
for iteration = 1:100
    %step 2
    for j=1:p
        %step 4
        sum_xv = 0;
        for i=1:n
            sum_xv = sum_xv + times(x(i),v(i,j));
        end
        z(j) = sigmoid_func(v_0(j)+sum_xv);
    end
    %step 5
    for k=1:m
        sum_xw = 0;
        for j=1:p
            sum_xw = sum_xw + times(z(j),w(j,k));
        end
        y(k) = sigmoid_func(w_0(k)+ sum_xw);
    end

hasil = round(y);
    
    %step 6
    for k = 1:m
        delta(k) = (t(k)-y(k))*(y(k)*(1-y(k)));
        for j = 1:p
            Delta_w(j,k) = alpha*delta(k)*z(j);
            Delta_w_0(k) = alpha*delta(k);
        end
    end

    %step 7
    for j = 1:p
        delta_in(j) = 0;
        for k = 1:m
            delta_in(j) = delta_in(j) + times(delta(k),w(j,k));
        end
        delta(j) = delta_in(j)*(z(j)*(1-z(j)));
        for i = 1:n
            Delta_v(i,j) = alpha*delta(j)*x(i);
            Delta_v_0(j) = alpha*delta(j);
        end
    end

    %step 8
    for k = 1:m
        for j = 1:p
            w(j,k) = w(j,k) + Delta_w(j,k);
        end
        w_0 = w_0 + Delta_w_0(k);

    end
    for j = 1:p
        for i = 1:n
            v(i,j) = v(i,j) + Delta_v(i,j);
        end
        v_0 = v_0 + Delta_v_0(j);
    end
end

%menyimpan bobot dan bias pada excel
xlswrite('Hasil_latih.xlsx',w,'sheet1','A1');
xlswrite('Hasil_latih.xlsx',w_0,'sheet2','A1');
xlswrite('Hasil_latih.xlsx',v,'sheet3','A1');
xlswrite('Hasil_latih.xlsx',v_0,'sheet4','A1');

data = xlsread('Hasil_latih.xlsx');
set(handles.uitable2,'Data', data);

% --- Executes on button press in pb_gscale.
function pb_gscale_Callback(hObject, eventdata, handles)
% hObject    handle to pb_gscale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% image = imread([formatfile, namafile]);
% img = imread(namafile);
img = handles.img;
namafile = handles.namafile;
threshold = graythresh (img);
img = ~imbinarize(img, threshold);
img = im2double(img);
handles.img = img;
axes(handles.axes2);
imshow(img);
guidata(hObject, handles);


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Mt=handles.Mt;
threshold=graythresh(Mt);
Mt=~imbinarize(Mt,threshold);
Mt=im2double(Mt);
Mt=bwareaopen(Mt,15);

image_data = imresize(Mt,[10,10]);
image1=image_data(:,:,1);
image1(image1==0)=-1;
x=reshape(image1,[1,100]);
namafilet=handles.namafilet;

%Pembentukan Pola Huruf untuk Pengenalan 
D1 = imread('D1.jpg');
threshold1=graythresh(D1);
P1=~imbinarize(D1,threshold1);
P1=im2double(P1);

E1 = imread('E1.jpg');
threshold2=graythresh(E1);
P2=~imbinarize(E1,threshold2);
P2=im2double(P2);

F1 = imread('F1.jpg');
threshold3=graythresh(F1);
P3=~imbinarize(F1,threshold3);
P3=im2double(P3);

D2 = imread('D2.jpg');
threshold4=graythresh(D2);
P4=~imbinarize(D2,threshold4);
P4=im2double(P4);

E2 = imread('E2.jpg');
threshold5=graythresh(E2);
P5=~imbinarize(E2,threshold5);
P5=im2double(P5);

F2 = imread('F2.jpg');
threshold6=graythresh(F2);
P6=~imbinarize(F2,threshold6);
P6=im2double(P6);

%target
t = ones(1,10);
if image == 'D1.jpg'
    t(2:10)=0;
elseif image=='E1.jpg'
    t(1)=0;
    t(3:10)=0;
elseif image=='F1.jpg'
    t(1:2)=0;
    t(4:10)=0;
elseif image == 'D2.jpg'
    t(2:10)=0;
elseif image=='E2.jpg'
    t(1)=0;
    t(3:10)=0;
elseif image=='F2.jpg'
    t(1:2)=0;
    t(4:10)=0;
end

% step 0
n = length(x);
m = length(t);
p = 5;
z = zeros(p,1);
v = xlsread('Hasil_latih.xlsx','sheet3','A1:E100');
w = xlsread('Hasil_latih.xlsx','sheet1','A1:J5');
v_0 = xlsread('Hasil_latih.xlsx','sheet4','A1:A5');
w_0 = xlsread('Hasil_latih.xlsx','sheet2','A1:A10');
theta = 0.05;
alpha = 1;
test = 0;

% step 1
% while test ~= 1
for iteration = 1:100
    %step 2
    for j=1:p
        %step 4
        sum_xv = 0;
        for i=1:n
            sum_xv = sum_xv + times(x(i),v(i,j));
        end
        z(j) = sigmoid_func(v_0(j)+sum_xv);
    end
    %step 5
    for k=1:m
        sum_xw = 0;
        for j=1:p
            sum_xw = sum_xw + times(z(j),w(j,k));
        end
        y(k) = sigmoid_func(w_0(k)+ sum_xw);
    end
end

hasil = round(y)

% Klasifikasi
D =[1,0,0,0,0,0,0,0,0,0];
E =[0,1,0,0,0,0,0,0,0,0];
F =[0,0,1,0,0,0,0,0,0,0];

if isequal(hasil,D)
    disp('D');
    set(handles.edit1,'String', 'Huruf D');
elseif isequal(hasil,E)
    set(handles.edit1,'String', 'Huruf E');
elseif isequal(hasil,F)
    set(handles.edit1,'String', 'Huruf F');
end


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[namafilet, formatfile] = uigetfile({'*.jpg'}, 'membuka gambar');
image = imread([formatfile, namafilet]);

Mt = imread(namafilet);
[baris,kolom] = size(Mt);
while (kolom>700)
    Mt = imresize(M,0.5);
    [baris,kolom] = size(Mt);
end
handles.Mt=Mt;
handles.namafilet = namafilet;
axes(handles.axes3);
imshow(Mt);
guidata(hObject, handles);

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Mt = handles.Mt;
threshold = graythresh(Mt);
Mt =~imbinarize(Mt,threshold);
Mt = im2double(Mt);
handles.Mt=Mt;
axes(handles.axes4);
imshow(Mt);
guidata(hObject, handles);
