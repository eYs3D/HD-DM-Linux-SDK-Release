# HD-DM-Linux-SDK-Release
## I.EtronDI Enviroment Setting (Ubuntu 18.04 x86_64)
Please run below command:
```sh 
cd Buid_Environment
sudo sh setup_env.sh 
```

## II.Build & Run DMPreview
There are two ways to compile the program:  
###### (1) Using QtCreator
-  1.1 Open QtCreator.  
-  1.2 Click "Open Project".  
![l-1](https://user-images.githubusercontent.com/13328289/120995299-dae02f80-c7b7-11eb-9ee5-f498787dcdd7.png)

-  1.3  Choose "./DMPreview/DMPreview.pro".  
![l-1](https://user-images.githubusercontent.com/13328289/121131954-184dc700-c863-11eb-8618-b80328e8a9c3.png)

- 1.4 Click "Configure Project".  
![ll-2](https://user-images.githubusercontent.com/13328289/121132040-35829580-c863-11eb-8b66-257928c0b8d9.png)

-  1.5 Select  Build/Clean All & Build/Rebuild All
![ll-3](https://user-images.githubusercontent.com/13328289/121132177-619e1680-c863-11eb-95fc-4f46e755fc34.png)

-  1.6 Select Build/Run to launch program
![ll-4](https://user-images.githubusercontent.com/13328289/121132224-74b0e680-c863-11eb-8ea9-593a7797fefa.png)


###### (2) Using QMake
-  Build & Run   
```sh 
#  Build:   
cd DMPreview  
qmake
make
```
![ll-5](https://user-images.githubusercontent.com/13328289/121132390-a629b200-c863-11eb-99ec-90e7af4e185c.png)
Compiled successfully.  
![ll-6](https://user-images.githubusercontent.com/13328289/121132428-b2ae0a80-c863-11eb-8c5a-ee0d00881bac.png)
Generate binary "DMPreview" under DMPreview folder.  
![ll-7](https://user-images.githubusercontent.com/13328289/121132451-b93c8200-c863-11eb-9f19-68254695ff1a.png)
Copy DMPreview to /bin folder  
```sh 
cp DMPreview ../bin
```
![ll-9](https://user-images.githubusercontent.com/13328289/121132538-da04d780-c863-11eb-94c1-cc186f7f470c.png)
Check /bin folder  
![ll-8](https://user-images.githubusercontent.com/13328289/121132561-df622200-c863-11eb-8c43-69c42bd4d412.png)
    
```sh     
cd bin
#  Rename:
mv DMPreview DMPreview_X86  
```
![ll-10](https://user-images.githubusercontent.com/13328289/121132706-0f112a00-c864-11eb-8a13-9daacf686e2e.png)
```sh   
#  Run:  
sh run_DMPreview_X86.sh  
```
![ll-11](https://user-images.githubusercontent.com/13328289/121132712-11738400-c864-11eb-836a-3bd190e336bd.png)
