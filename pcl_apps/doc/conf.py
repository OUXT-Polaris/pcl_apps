#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2019 OUXT Polaris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.mathjax',
]

templates_path = ['_templates']

source_suffix = '.rst'

master_doc = 'index'

project = 'pcl_apps'
copyright = '2019, Masaya Kataoka'
author = 'Masaya Kataoka'

version = '0.0.0'

release = '0.0.0'

language = None

exclude_patterns = ['_build']

pygments_style = 'sphinx'

todo_include_todos = True

html_theme = 'sphinx_rtd_theme'

html_static_path = ['_static']

htmlhelp_basename = 'pcl_appsdoc'

latex_elements = {
}

latex_documents = [
  (master_doc, 'pcl_apps.tex', 'pcl\\_apps Documentation',
   'Masaya Kataoka', 'manual'),
]

man_pages = [
    (master_doc, 'pcl_apps', 'pcl_apps Documentation',
     [author], 1)
]

texinfo_documents = [
  (master_doc, 'pcl_apps', 'pcl_apps Documentation',
   author, 'pcl_apps', 'One line description of project.',
   'Miscellaneous'),
]

epub_title = project
epub_author = author
epub_publisher = author
epub_copyright = copyright

epub_exclude_files = ['search.html']

intersphinx_mapping = {'https://docs.python.org/': None}
