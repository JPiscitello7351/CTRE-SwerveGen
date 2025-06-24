# Viewing the Documentation

The docs hosted from this repository are available at: [litemage.github.io/CTRE-SwerveGen-Test/](https://litemage.github.io/CTRE-SwerveGen-Test/)

**Soon these docs will move back to the repo they were forked from**

# Adding Files to Docs

You can add files to the documentation by creating a Markdown file in the `docs/` directory, and then editing `mkdocs.yml`.

To edit `mkdocs.yml` to tell mkdocs where to find your new file, scroll down until you see a comment laballed "this is where you add your pages". Follow the format for all the other entries.

# Building Documentation

This project uses [material for mkdocs](https://squidfunk.github.io/mkdocs-material/) for building documentation.

It also requires the [mkdoxy](https://github.com/JakubAndrysek/MkDoxy-demo) pip package and the [doxygen](https://www.doxygen.nl/index.html) executable. 

The documentation is capable of documenting any docstring in any file within the `src/main` directory.

Follow the instructions below to open the docs, or install the necessary tools and build the documentation.

## Install Tooling & Build

1. You need **Python** installed and available in your path for this to work. You can get this from the [Microsoft store](https://apps.microsoft.com/detail/9nrwmjp3717k?hl=en-us&gl=US) or from the [python foundation](https://www.python.org/downloads/). Make sure you **add it to your path** when asked in the installer.
2. Install [Doxygen](www.doxygen.nl/download.html) and also add this to your path. The specific directory you'll need to add is: `C:\Program Files\doxygen\bin`. If help is needed, follow the instructions [here](https://windowsloop.com/how-to-add-to-windows-path/) (scroll down to "Add directory or program to Windows PATH")
3. Open this repository
4. Run the following commands:

```
python -m venv ./.venv
./.venv/Scripts/activate
pip install -r requirements.txt
mkdocs serve
```

5. Open a web browser and open [localhost:8000](https://localhost:8000/) to view documentation
6. Use `mkdocs build` to build the documentation into a folder at `Docs/site/` if you want to

## Troubleshooting

**Running scripts is disabled on this system**

This may occur while trying to run `./.venv/Scripts/activate`. You can enable running scripts by following the instructions [here](https://stackoverflow.com/a/64633728)

**Documentation fails to build**

Check to make sure that `python`, `mkdoxy`, and `doxygen` were installed properly and added to the PATH environment variable you can test this by typing `doxygen -v` or `python --version` into a terminal, and if a version is printed, it is installed correctly.