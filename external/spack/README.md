
# Seahowl – Environnement Spack

## Prerequisit
- Linux
- Spack >= 0.22
- gcc or clang

## Spack Installation
```bash
git clone https://github.com/spack/spack.git
source spack/share/spack/setup-env.sh
```

## Install dependancies
```bash
cd seahowl-spack-env
spack env activate .
spack install
```
## Build Sehaowl

Build seahowl with preset full/Spack or full/Spack/debug

```bash
mkdir build
cmake --preset=full/Spack ..
ninja -j8
```
