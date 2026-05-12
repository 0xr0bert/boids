project(
    'boids',
    'cpp',
    version : '1.0.0',
    default_options : ['warning_level=3', 'cpp_std=c++20']
)

boids = executable(
    'boids',
    sources: [
        'src/main.cpp',
        'src/simulation.cpp'
    ],
    install : false,
    include_directories: include_directories('src')
)
