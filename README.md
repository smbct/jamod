# setup

Attempt ja modding. Original repo: <https://github.com/JACoders/OpenJK/>

```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=ja_parent_directory -DBuildMPEngine=OFF -DBuildMPDed=OFF 
make -j5 && make install
```

To activate the kinect mod, use cmake option `-DKinectMod=ON`.

Running: 

In the GameData directory:

```
./openjk_sp_x86
```

If no save file: open consol ("small 2" key) and type `devmap t1_sour`
Once a quicksave is performed: `./openjk_sp_x86 load auto`
Or a normal save: `./openjk_sp_x86 load jedi_00`