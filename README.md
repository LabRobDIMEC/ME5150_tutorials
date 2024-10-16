# ME5150_tutorials
## Guía de instalación:
- Clonar repositorio en tu computador: `git clone https://github.com/LabRobDIMEC/ME5150_tutorials.git`
- Abrir consola de anaconda
- Actualizar anaconda si ya tienes una versión antigua: `conda update --all`
- Navegar hasta la carpeta ME5150_tutorials usando el comando `cd`
- Crear ambiente virtual de anaconda con archivo .yml: `conda env create -f environment.yml`
- Desde Visual Studio Code instalar extensión de Python
- Selección interprete de python `3.8.16 ('robotica': conda)` en la barra inferior derecha.
- Correr los ejemplos & disfrutar

## Actualizar contenido del repositorio:
- Abrir alguna terminal
- Navegar hasta la carpeta ME5150_tutorials usando el comando `cd`
- Correr `git pull`

Si tienen algún archivo propio, asegúrense de guardarlo en una carpeta fuera del repositorio.

## Actualizar requerimientos del ambiente:
- Asegurarse de tener el repositorio actualizado `git pull`
- Abrir consola de anaconda
- Navegar hasta la carpeta ME5150_tutorials usando el comando `cd` (si es que no se está en ella)
- Activar ambiente virtual `conda activate robotica`
- Correr `conda env update --file environment.yml  --prune`
- Listo
