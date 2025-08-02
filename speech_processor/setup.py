from setuptools import setup

package_name = 'speech_processor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'requests',
        'pydub',
    ],
    zip_safe=True,
    maintainer='Nuralem Abizov',
    maintainer_email='abizov94@gmail.com',
    description='ROS2 package for speech processing including TTS and audio management for Go2 robot',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'tts_node = speech_processor.tts_node:main',
            'speech_synthesizer = speech_processor.speech_synthesizer_node:main',
            'audio_manager = speech_processor.audio_manager_node:main',
        ],
    },
) 