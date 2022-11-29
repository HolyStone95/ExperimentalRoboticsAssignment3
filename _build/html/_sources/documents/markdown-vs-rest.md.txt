
# MarkDown Vs reStructured Text

see [this](https://myst-parser.readthedocs.io/en/latest/syntax/roles-and-directives.html)

## ToC tree

in reST:

```
.. toctree::
	:maxdepth: 1
	:caption: ToC title
	:titlesonly:
	:glob:
	:hidden:
	
	...
	link text <./path/link.format>
	...
```

in MD-MyST:

```
\```{toctree}
---
maxdepth: 2
caption: ToC title
titlesonly:
glob: 
hidden: 
---

link text <./path/link.format>

\```
```

## reST codeblocks

```
.. code-block::
	:caption: a code block caption
	
	import rospy 
	# ...
```
