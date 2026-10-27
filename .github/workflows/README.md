Ok.
Now I failed to mention this originally, 
I'm new to CI development as well.
First off there is no CI.

second off uv sync --extra dev doesn't work here
what does that mean?

It means the command asks uv for something your project doesn’t define.
A CI workflow runs the same install command you would. 
If it says uv sync --extra dev and the branch doesn’t define dev, the job fails at the first step.

The branch and the subbranches need an update to pyproject.toml

So...I think I should do that.
So...
I added some other things to the pyproject.toml so that the rest of this stuff will run with
uv sync --extra dev
and then the actual dependencies

Now, let's work on the actual ci.yml file
