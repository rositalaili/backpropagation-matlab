clc;
clear all;

%input gambar
image = input('Masukkan nama file gambar:');
imshow(image);
M = imread(image);
[baris,kolom]=size(M);

%Jika skala gambar berukuran besar akan dilakukan resize
while(kolom>700)
    M=imresize(M,0.5);
    [baris,kolom]=size(M);
end
%Pengubahan RGB ke Grayscale
if size(M,3)==3
    G=rgb2gray(M);
end
threshold=graythresh(M);
M=~imbinarize(M,threshold);
M=im2double(M);
M=bwareaopen(M,15);

image_data = imresize(M,[10,10]);
image1=image_data(:,:,1);
image1(image1==0)=-1;
x=reshape(image1,[1,100]);

%%
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
 %% Algoritma BP
% step 0
n = length(x);
m = length(t);
p = 5; %banyak neuron di hidden layer
z = zeros(p,1); %hidden layer
v = xlsread('Hasil_latih.xlsx','sheet3','A1:E100');
w = xlsread('Hasil_latih.xlsx','sheet1','A1:J5');
v_0 = xlsread('Hasil_latih.xlsx','sheet4','A1:A5');
w_0 = xlsread('Hasil_latih.xlsx','sheet2','A1:A10');
theta = 0.05;
alpha = 1;
test = 0;
% step 1
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
%% Cetak bobot dan bias

%menyimpan bobot dan bias pada excel
xlswrite('Hasil_latih.xlsx',w,'sheet1','A1');
xlswrite('Hasil_latih.xlsx',w_0,'sheet2','A1');
xlswrite('Hasil_latih.xlsx',v,'sheet3','A1');
xlswrite('Hasil_latih.xlsx',v_0,'sheet4','A1');

disp('Update bias dan bobot pada unit output')
disp('bobot: ')
disp(w)
disp('bias: ')
disp(w_0)

disp('Update bias dan bobot pada unit hidden')
disp('bobot: ')
disp(v)
disp('bobot: ')
disp(v_0)

disp('output: ')
disp(hasil)