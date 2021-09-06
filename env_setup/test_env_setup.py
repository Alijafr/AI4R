
import pkg_resources
import platform
import traceback
import tkinter

COURSE_PYTHON_VERSION = '3.9'
COURSE_MIN_RECOMMENDED_PYTHON_VERSION = '3.6'

def verify_python_installation():
    print('\n< Verifying python installation... >')
    python_version = platform.python_version()
    print(f'\tYour python version: {python_version}')
    if python_version[:3] != COURSE_PYTHON_VERSION:
        warning_msg = f'\t~ Your python version is not the same as the version used by the autograder ({COURSE_PYTHON_VERSION}).\n'\
                      f'\t~ The minimum recommended version is {COURSE_MIN_RECOMMENDED_PYTHON_VERSION}.'
        print(warning_msg)

def verify_libraries_installation():
    print('\n< Verifying libraries installation... >')
    with open('rait_env.yml', 'r') as f:
        yaml_contents = f.readlines()
    dependencies = [line.strip('- \n') for line in yaml_contents[yaml_contents.index('dependencies:\n')+2:] if ':' not in line]
    try:
        pkgs = set([str(pkg) for pkg in pkg_resources.require(dependencies)])
        for pkg in pkgs:
            pkg_name = pkg.split()[0]
            if pkg_name in dependencies:
                print(f'\t{pkg}')
        print('\t✅  All libraries installed!')
    except pkg_resources.DistributionNotFound as e:
        print('\t⚠️ Missing library')
        print(traceback.format_exc())
    except Exception as e:
        print('\t❌️ Error')
        print(traceback.format_exc())

def verify_unicode_display():
    print('\n< Verifying unicode display... >')
    print(f'\tYou should see 8 arrows here: ↖↗↘↙←→↑↓\n'
          f"\tIf you don't see the arrows above then you should install the symbola font and run this script again.")

def verify_tkinter_installation():
    print('\n< Verifying tkinter installation... >')
    print('\tA GUI window should appear with a "click me" and "quit" button demonstrating that tkinter is working properly.')
    tkinter._test()


if __name__ == '__main__':
    verify_python_installation()
    verify_libraries_installation()
    verify_unicode_display()
    verify_tkinter_installation()
