"""Checks that a problem report has the files it needs.

Run by .github/workflows/issue-logs.yml when an issue is opened, edited or
reopened. A tracking or calibration problem needs vrserver.txt and a
diagnostics file. A setup or program problem needs what its "Where is the
problem?" answer names (AREA_NEEDS): the installer writes no log, so for it,
and for a Nova Calibrator that never started, the window or error text stands
in. Any other issue, one opened through the API included, is held to the
tracking form's rules. In either form, a headset or connection answered
"Other" needs its name typed in the field after it. The forms only make fields non-empty; this checks that
each holds an upload with the right name. A report that lacks something gets
the "needs logs" label and one comment naming what is missing, which is
updated as the report is edited and removed with the label once nothing is.
Translation corrections, closed issues and issues by people with write access
are left alone.

`python tools/ci/issue-logs.py --check <file>` prints what a body is missing,
without GitHub.
"""

import json
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request

LABEL = "needs logs"
LABEL_COLOR = "fbca04"
LABEL_DESCRIPTION = "The report does not have the files that its form asks for"
MARKER = "<!-- nova-calibrator:issue-logs -->"
TRUSTED = {"OWNER", "MEMBER", "COLLABORATOR"}

# Field labels of the forms in .github/ISSUE_TEMPLATE; change them together.
# A field only the translation form has:
TRANSLATION_HEADING = "Text on the screen now"
# The setup form's area question, the field for the installer's or an error's
# text, and what each answer needs:
AREA_HEADING = "Where is the problem?"
WINDOW_TEXT_HEADING = "Text in the installer window or the error message"
AREA_NEEDS = {
    "The installer or the uninstaller": ["window-text"],
    "Nova Calibrator does not start": ["window-text"],
    "SteamVR shows an error or does not load the Nova Calibrator driver": ["vrserver", "diagnostics"],
    "Updates": ["diagnostics"],
    "The Nova Calibrator window, its settings or its text": ["diagnostics"],
    "Other": ["diagnostics"],
}
TRACKING_NEEDS = ["vrserver", "diagnostics"]
# A dropdown answered "Other" needs the name typed in the field after it,
# in either problem form.
OTHER_NAMES = {
    "other-headset": ("Headset", "Name of the other headset"),
    "other-connection": ("Connection to the PC", "Name of the other connection"),
}

# An upload is linked as https://github.com/user-attachments/files/<id>/<name>,
# or on older reports https://github.com/<owner>/<repo>/files/<id>/<name>.
ATTACHMENT = re.compile(
    r"https://github\.com/(?:user-attachments/files|[^/\s()]+/[^/\s()]+/files)/\d+/([^\s()<>\]\"']+)",
    re.IGNORECASE)
# vrserver.txt, vrserver.previous.txt, or a copy such as vrserver (1).txt,
# which GitHub uploads as vrserver.1.txt.
VRSERVER = re.compile(r"^vrserver[\w.-]*\.(?:txt|log)$", re.IGNORECASE)
# The diagnostics file, under the name before 1.2.0 too.
DIAGNOSTICS = re.compile(r"^(?:Nova|Quest)Calibrator-diagnostics-[\w.-]*\.txt$", re.IGNORECASE)

MISSING_TEXT = {
    "vrserver": '"vrserver.txt", in the "logs" folder of Steam (usually `C:\\Program Files (x86)\\Steam\\logs`).',
    "diagnostics": 'A diagnostics file. In Nova Calibrator, open "Settings" and select "Save diagnostics file". '
                   'Nova Calibrator also writes one in `%LOCALAPPDATA%\\NovaCalibrator\\diagnostics` '
                   "when SteamVR stops and when Nova Calibrator closes.",
    "window-text": 'The text in the installer window or the error message, in the "' + WINDOW_TEXT_HEADING
                   + '" box. Copy the text into the box, or attach a screenshot.',
    "other-headset": 'The name of your headset, in the "Name of the other headset" box.',
    "other-connection": 'The name of your connection to the PC, in the "Name of the other connection" box.',
}


def sections(body):
    """The form's fields: each "### <label>" heading over its answer."""
    found, label, lines = {}, None, []
    for line in (body or "").splitlines():
        if line.startswith("### "):
            if label is not None:
                found[label] = "\n".join(lines).strip()
            label, lines = line[4:].strip(), []
        elif label is not None:
            lines.append(line)
    if label is not None:
        found[label] = "\n".join(lines).strip()
    return found


def answered(text):
    # An optional field left empty renders as "_No response_".
    return bool(text) and text != "_No response_"


def needed_files(body):
    """None for a report that needs nothing, else what it must have."""
    fields = sections(body)
    if TRANSLATION_HEADING in fields:
        return None
    if AREA_HEADING in fields:
        # An answer not in the list (a hand-made body, or options changed
        # without AREA_NEEDS) needs the most a setup problem can.
        return AREA_NEEDS.get(fields[AREA_HEADING], ["vrserver", "diagnostics"])
    return TRACKING_NEEDS


def attachment_names(body):
    return [urllib.parse.unquote(m.group(1)) for m in ATTACHMENT.finditer(body or "")]


def missing_files(body):
    needs = needed_files(body)
    if needs is None:
        return []
    names, fields = attachment_names(body), sections(body)
    have = {
        "vrserver": any(VRSERVER.match(n) for n in names),
        "diagnostics": any(DIAGNOSTICS.match(n) for n in names),
        "window-text": answered(fields.get(WINDOW_TEXT_HEADING)),
    }
    missing = [need for need in needs if not have[need]]
    for item, (dropdown, name_field) in OTHER_NAMES.items():
        if fields.get(dropdown) == "Other" and not answered(fields.get(name_field)):
            missing.append(item)
    return missing


def comment_text(missing):
    items = "\n".join("- " + MISSING_TEXT[m] for m in missing)
    return (MARKER + "\n"
            "This report does not have all of the items that we need to examine the problem.\n"
            "Edit the report and add these items:\n\n" + items + "\n\n"
            "Drag each file into the report. Attach each file as it is, not in a .zip file.\n"
            'The "' + LABEL + '" label goes away when the report has all of the items.')


class GitHub:
    def __init__(self, repo, token):
        self.base = "https://api.github.com/repos/" + repo
        self.token = token

    def call(self, method, path, data=None, allow=()):
        request = urllib.request.Request(
            self.base + path, method=method,
            data=None if data is None else json.dumps(data).encode(),
            headers={"Authorization": "Bearer " + self.token,
                     "Accept": "application/vnd.github+json",
                     "X-GitHub-Api-Version": "2022-11-28"})
        try:
            with urllib.request.urlopen(request) as response:
                text = response.read()
                return json.loads(text) if text else None
        except urllib.error.HTTPError as e:
            if e.code in allow:
                return None
            raise

    def marker_comments(self, number):
        found, page = [], 1
        while True:
            batch = self.call("GET", f"/issues/{number}/comments?per_page=100&page={page}")
            found += [c for c in batch
                      if c["user"]["login"] == "github-actions[bot]" and MARKER in (c["body"] or "")]
            if len(batch) < 100:
                return found
            page += 1


def run_on_event():
    with open(os.environ["GITHUB_EVENT_PATH"], encoding="utf-8") as f:
        issue = json.load(f)["issue"]
    number, body = issue["number"], issue.get("body") or ""
    if issue.get("state") != "open":
        return print(f"#{number} is closed")
    if issue.get("author_association") in TRUSTED:
        return print(f"#{number} is by {issue['author_association']}")
    if needed_files(body) is None:
        return print(f"#{number} is a translation correction")

    gh = GitHub(os.environ["GITHUB_REPOSITORY"], os.environ["GITHUB_TOKEN"])
    missing = missing_files(body)
    comments = gh.marker_comments(number)
    labelled = any(label["name"] == LABEL for label in issue.get("labels", []))
    print(f"#{number}: needs {needed_files(body)}, attachments {attachment_names(body)}, missing {missing}")

    if missing:
        gh.call("POST", "/labels", {"name": LABEL, "color": LABEL_COLOR, "description": LABEL_DESCRIPTION},
                allow=(422,))
        if not labelled:
            gh.call("POST", f"/issues/{number}/labels", {"labels": [LABEL]})
        text = comment_text(missing)
        if not comments:
            gh.call("POST", f"/issues/{number}/comments", {"body": text})
        elif comments[0]["body"] != text:
            gh.call("PATCH", f"/issues/comments/{comments[0]['id']}", {"body": text})
        for extra in comments[1:]:
            gh.call("DELETE", f"/issues/comments/{extra['id']}", allow=(404,))
    else:
        if labelled:
            gh.call("DELETE", f"/issues/{number}/labels/" + urllib.parse.quote(LABEL), allow=(404,))
        for comment in comments:
            gh.call("DELETE", f"/issues/comments/{comment['id']}", allow=(404,))


def main(argv):
    if len(argv) == 3 and argv[1] == "--check":
        with open(argv[2], encoding="utf-8") as f:
            body = f.read()
        print("needs:", needed_files(body))
        print("attachments:", attachment_names(body))
        print("missing:", missing_files(body) or "nothing")
        return 0
    run_on_event()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
