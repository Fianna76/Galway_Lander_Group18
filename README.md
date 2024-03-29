# Basic Set-Up 

## Getting Github set up

First, install the newest version of [Git](https://git-scm.com/download) for your device
Once you have Git installed, open a terminal 

Go to Github and sign in/register your account - keep this browser open because you'll need to sign in again

### Fork this repo

Follow the steps listed in [this](https://www.freecodecamp.org/news/how-to-make-your-first-pull-request-on-github-3/) guide, and set up your own local branch of the project. You can name your branch whatever you'd like but I'd reccomend just your first name for this project

It's important to keep your own work confined to your own branch, and then merge changes to the master later
Do **not** force push anything

Github isn't an auto sync software (at least not with the IDEs we're using), you need to be constantly *push*ing your new code into the shared repo, and *pull*ing the code other people worked on to stay up to date. 
If we all work out of the master branch, this will inevitibly lead to errors - stay in your own branch until it's the proper time to merge

### Using git
Each time you change a file, you use the `git add` command on it to add it to your next commit.
Simply saving a file with changes on it is *not* enough for it to be uploaded with commit - git needs to be explicitly told what you want to commit

When you're ready to upload your latest changes to your branch, use the command `git commit -m "[COMMENTS ON COMMIT]`, and then finally `git push`

Again, *please* only commit to your branch