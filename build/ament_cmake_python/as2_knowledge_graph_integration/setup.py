from setuptools import find_packages
from setuptools import setup

setup(
    name='as2_knowledge_graph_integration',
    version='0.0.0',
    packages=find_packages(
        include=('as2_knowledge_graph_integration', 'as2_knowledge_graph_integration.*')),
)
