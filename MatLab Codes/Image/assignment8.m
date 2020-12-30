clearvars -global;
clc;
clear all;
image=imread('columns.png');
bw=rgb2gray(image);         %Image converted to grayscale
[R,C]=size(bw);
image_1D=reshape(bw.',1,[]);     %Convert image to 1-D array
inputMatrix=de2bi(image_1D,'left-msb');   %Convert grayscale intensities to 8-bit binary
im_uncoded_1D = reshape(inputMatrix.',1,[]);        %UNcoded 1-D image
Image_4bit=reshape(im_uncoded_1D,4,[]).';          %4-bit image bit       
%%          G & H Matrices
P=[0 1 1;1 0 1;1 1 0;1 1 1];
G=[eye(4),P];           %Generator Matrix
H=[P',eye(3)];

%%            Adding noise to Uncoded Image
p=[0.001, 0.01, 0.1, 0, 0.5, 0.9, 0.99, 0.999];
error_matrix_uncoded=uint8(zeros(7,8*R*C));
noisy_im_uncoded=uint8(zeros(7,8*R*C));
recon_image1=uint8(zeros(R*C,8,7));
recon_image2=uint8(zeros(R*C,1,7));
recon_image_uncoded_final=uint8(zeros(R,C,7));
for i=1:7
    error_matrix_uncoded(i,:)=uint8(randsrc(1,8*R*C,[1,0;p(i),(1-p(i))]));
    noisy_im_uncoded(i,:)=uint8(mod(im_uncoded_1D+error_matrix_uncoded(i,:),2));
    recon_image1(:,:,i)=reshape(noisy_im_uncoded(i,:),8,[]).';
    recon_image2(:,:,i)=uint8(bi2de(recon_image1(:,:,i),'left-msb'));          %Convert to grayscale values
    recon_image_uncoded_final(:,:,i)=reshape(recon_image2(:,:,i).',C,[]).';     %Reshape into R X C matrix 
end
%%      Perform channel Coding
imCoded=uint8(mod(double(Image_4bit)*G,2));
imCoded_1D=reshape(imCoded.',1,[]);                           %Coded 1-D image
%%
%       for Coded image
error_matrix=uint8(zeros(7,14*R*C));
noisy_coded=uint8(zeros(7,14*R*C));

for i=1:7               %For 7 different values of p
    error_matrix(i,:)=uint8(randsrc(1,14*R*C,[1,0;p(i),(1-p(i))]));
    noisy_coded(i,:)=mod(double(error_matrix(i,:)+imCoded_1D),2);
end

%%   Reconstructing coded image
%Declaring various matrices
imRxed=uint8(zeros(2*R*C,7,7));
Syndrome=uint8(zeros(2*R*C,3,7));
Index=uint8(zeros(2*R*C,1,7));
corrected_coded_image=uint8(zeros(2*R*C,7,7));
decoded_image=uint8(zeros(2*R*C,4,7));
decoded_image1=uint8(zeros(1,8*R*C,7));
decoded_image_reshaped=uint8(zeros(R*C,8,7));
decoded_image_gray_1D=uint8(zeros(R*C,1,7));
decoded_image_final=uint8(zeros(R,C,7));


    %%          Reshape noisy image
decoding_matrix=[1,0,0,0,0,0,0;0,1,0,0,0,0,0;0,0,1,0,0,0,0;0,0,0,1,0,0,0].'; %We need to take 1,2,3,4 bits from the codeword obtained
  for i=1:7
    imRxed(:,:,i)=reshape(noisy_coded(i,:),7,[]).';
    corrected_coded_image(:,:,i)=imRxed(:,:,i);
    Syndrome(:,:,i)=uint8(mod(double(imRxed(:,:,i))*H',2));
    Index(:,:,i)=uint8(bi2de(Syndrome(:,:,i),'right-msb'));        %Checking error position from Syndrome
    for j=1:2*R*C
        if Index(j,1,i)==0              %In case of no error
        %
        else
            corrected_coded_image(j,Index(j,1,i),i)=uint8(mod(corrected_coded_image(j,Index(j,1,i),i)+1,2));
        end
    end
    decoded_image(:,:,i)=uint8(mod(double(corrected_coded_image(:,:,i))*decoding_matrix,2));
    decoded_image1(:,:,i) = reshape(decoded_image(:,:,i).',1,[]);
    decoded_image_reshaped(:,:,i)=reshape(decoded_image1(:,:,i),8,[]).';
    decoded_image_gray_1D(:,:,i)=uint8(bi2de(decoded_image_reshaped(:,:,i),'left-msb'));
    decoded_image_final(:,:,i)=reshape(decoded_image_gray_1D(:,:,i).',C,[]).';
  end
%%      Sachin plots 

%Plotting the original image, the reconstructed image after coding and 
%the noisy uncoded image.
% f3=figure(3);
% subplot(2,4,5),imshow(RGB_image),title('(Original colour image)');
% subplot(2,4,1),imshow(Image),title('(Original B/W image)');
% subplot(2,4,6),imshow(uint8(decoded_image_final(:,:,5))),title('Reconstucted coded image (Pb=0.9)');
% subplot(2,4,2),imshow(uint8(recon_image_final(:,:,5))),title('Reconstucted Uncoded image (Pb=0.9)');
% subplot(2,4,7),imshow(uint8(decoded_image_final(:,:,6))),title('Reconstucted coded image (Pb=0.99)');
% subplot(2,4,3),imshow(uint8(recon_image_final(:,:,6))),title('Reconstucted coded image (Pb=0.99)');
% subplot(2,4,8),imshow(uint8(decoded_image_final(:,:,7))),title('Reconstucted coded image (Pb=0.999)');
% subplot(2,4,4),imshow(uint8(recon_image_final(:,:,7))),title('Reconstucted coded image (Pb=0.999)');
% 
% f2=figure(2);
% subplot(2,3,4),imshow(RGB_image),title('(Original colour image)');
% subplot(2,3,1),imshow(Image),title('(Original B/W image)');
% subplot(2,3,5),imshow(uint8(decoded_image_final(:,:,3))),title('Reconstucted coded image (Pb=0.1)');
% subplot(2,3,2),imshow(uint8(recon_image_final(:,:,3))),title('Reconstucted Uncoded image (Pb=0.1)');
% subplot(2,3,6),imshow(uint8(decoded_image_final(:,:,4))),title('Reconstucted coded image (Pb=0.5)');
% subplot(2,3,3),imshow(uint8(recon_image_final(:,:,4))),title('Reconstucted coded image (Pb=0.5)');
% 
% f1=figure(1);
% subplot(2,3,4),imshow(RGB_image),title('(Original colour image)');
% subplot(2,3,1),imshow(Image),title('(Original B/W image)');
% subplot(2,3,5),imshow(uint8(decoded_image_final(:,:,1))),title('Reconstucted coded image (Pb=0.001)');
% subplot(2,3,2),imshow(uint8(recon_image_final(:,:,1))),title('Reconstucted Uncoded image (Pb=0.001)');
% subplot(2,3,6),imshow(uint8(decoded_image_final(:,:,2))),title('Reconstucted coded image (Pb=0.01)');
% subplot(2,3,3),imshow(uint8(recon_image_final(:,:,2))),title('Reconstucted coded image (Pb=0.01)');
% 
% f1.WindowState = 'maximized';
% f2.WindowState = 'maximized';
% f3.WindowState = 'maximized';
  
  %%                      Plots
f1=figure;
subplot(2,4,1);
imshow(image),title('(Original colour image)');
subplot(2,4,5);
imshow(bw),title('(Original Grayscale image)');
subplot(2,4,2);
imshow(uint8(recon_image_uncoded_final(:,:,1))),title('Reconstucted Uncoded image (Pe=0.001)');
subplot(2,4,3);
imshow(uint8(recon_image_uncoded_final(:,:,2))),title('Reconstucted Uncoded image (Pe=0.01)');
subplot(2,4,4);
imshow(uint8(recon_image_uncoded_final(:,:,3))),title('Reconstucted Uncoded image (Pe=0.1)');
subplot(2,4,6);
imshow(uint8(decoded_image_final(:,:,1))),title('Reconstucted Coded image (Pe=0.001)');
subplot(2,4,7);
imshow(uint8(decoded_image_final(:,:,2))),title('Reconstucted Coded image (Pe=0.01)');
subplot(2,4,8);
imshow(uint8(decoded_image_final(:,:,3))),title('Reconstucted Coded image (Pe=0.1)');

f2=figure(2);
 subplot(2,3,1),imshow(image),title('(Original colour image)');
 subplot(2,3,4),imshow(bw),title('(Original grayscale image)');
 subplot(2,3,2),imshow(uint8(recon_image_uncoded_final(:,:,4))),title('Reconstucted Uncoded image (Pe=0)');
 subplot(2,3,3),imshow(uint8(recon_image_uncoded_final(:,:,5))),title('Reconstucted Uncoded image (Pb=0.5)');
 subplot(2,3,5),imshow(uint8(decoded_image_final(:,:,4))),title('Reconstucted coded image (Pb=0)');
 subplot(2,3,6),imshow(uint8(decoded_image_final(:,:,5))),title('Reconstucted coded image (Pb=0.5)');

 f3=figure(3);
 subplot(2,3,1),imshow(image),title('(Original colour image)');
 subplot(2,3,4),imshow(bw),title('(Original grayscale image)');
 subplot(2,3,2),imshow(uint8(recon_image_uncoded_final(:,:,6))),title('Reconstucted Uncoded image (Pe=0.9)');
 subplot(2,3,3),imshow(uint8(recon_image_uncoded_final(:,:,7))),title('Reconstucted Uncoded image (Pb=0.99)');
 subplot(2,3,5),imshow(uint8(decoded_image_final(:,:,6))),title('Reconstucted coded image (Pb=0.9)');
 subplot(2,3,6),imshow(uint8(decoded_image_final(:,:,7))),title('Reconstucted coded image (Pb=0.99)');
