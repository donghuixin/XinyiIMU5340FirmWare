import * as vscode from 'vscode';
import fetch from 'node-fetch';

const API_URL = 'https://api.musingdrift.com/your/api/endpoint'; // 修改为实际 endpoint
const API_KEY = 'sk-aqqRTWU6YZ9iY1iLPzyQZepJoU8IjdcPpQFKKYXopcRTfOAX';

async function callMusingdriftAPI(prompt: string): Promise<string> {
  const response = await fetch(API_URL, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${API_KEY}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ prompt }) // 根据实际 API body 调整
  });
  if (!response.ok) {
    return `API Error: ${response.status} ${response.statusText}`;
  }
  const data = await response.json();
  // 假设返回 { result: "..." }，如有不同请调整
  return data.result || JSON.stringify(data);
}

export function activate(context: vscode.ExtensionContext) {
  let disposable = vscode.commands.registerCommand('extension.askMusingdrift', async () => {
    const editor = vscode.window.activeTextEditor;
    if (!editor) {
      vscode.window.showInformationMessage('No active editor');
      return;
    }
    const prompt = editor.document.getText(editor.selection) || await vscode.window.showInputBox({ prompt: 'Enter your prompt for Musingdrift AI' });
    if (!prompt) {
      vscode.window.showInformationMessage('No prompt provided');
      return;
    }
    vscode.window.withProgress({ location: vscode.ProgressLocation.Notification, title: 'Musingdrift AI is thinking...' }, async () => {
      const result = await callMusingdriftAPI(prompt);
      editor.edit(editBuilder => {
        editBuilder.insert(editor.selection.end, '\n' + result);
      });
    });
  });
  context.subscriptions.push(disposable);
}

export function deactivate() {}
