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

Now, let's work on the actual ci.yml file.

Ohk...I just saw that email. 
```
The PR text appears to be the output of an AI agent without even a review to catch things like:
The reason I'm creating this file is because my dumaas can't keep track of shit in my working memory.

The text also implies that the code was largely, if not entirely, written by the agent. The .md file containing the vulgar ramblings of the AI is included in the PR, which would look incredibly unprofessional if it were to be merged. What's more, it's making changes to QUEST_algorithm.py which is outside the scope of this issue.

A large part of the purpose of this project is to practice the concepts being used. I will not merge code that is blatantly generated and certainly not if it apparently hasn't even undergone a cursory review by the person submitting.

I am offering my time to this project as a mentor. If we wanted a slopped-together simulator I could sic an agent of my own on it and save everybody the hassle.

Refer to the AI Usage Policy in README.md. It will soon be moved to a CONTRIBUTORS.md file should you ever need to return to it in the future.
```

This feels horrible.
Because I genuinely did write that entire thing myself.
Honestly, the only use that I had was ask copilot to write the thing in a proper .md format so that it was still documentation based. And I did go through the tests as well to figure out if it changed anything in my test files...it must have changed the original QUEST files without me realising it.

The vulgarity was a part of it, to show that it was a human writing that. Point noted, I'll try and stick to being as professional as possible. I write those markdowns as a way of orienting myself and keeping track. The reason I stopped being professional is to show that it wasn't wrtitten by AI.
Fair enough.

Damn, now I'm depressed.
And slightly mortified. I'm sorry. 


