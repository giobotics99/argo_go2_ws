#!/usr/bin/env python3
"""Chat with Gemma (llama.cpp on port 8000) from the terminal, to try prompts by hand.

    python3 tools/gemma_chat.py
    python3 tools/gemma_chat.py --system "Sei Argo, un cane robot simpatico. Racconta barzellette sul caffè."
    python3 tools/gemma_chat.py --temperature 0.9 --system-file mio_prompt.txt

Commands while chatting:
    /reset          forget the conversation (keeps the system prompt)
    /system TESTO   change the system prompt (and reset)
    /temp 0.7       change the temperature
    /quit           exit (also Ctrl+D)
No ROS needed: it talks to llama.cpp directly, like gemma_server.py does.
"""
import argparse
import json
import time
import urllib.request

DEFAULT_SYSTEM = ("Sei Argo, un cane robot simpatico che vive nel laboratorio di robotica. "
                  "Rispondi sempre in italiano, in prima persona, con frasi brevi.")


def ask(api, model, messages, temperature, max_tokens, timeout):
    payload = {'model': model, 'messages': messages, 'temperature': temperature,
               'max_tokens': max_tokens, 'stream': False}
    req = urllib.request.Request(f'{api}/chat/completions', data=json.dumps(payload).encode('utf-8'),
                                 headers={'Content-Type': 'application/json'}, method='POST')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # no proxy for localhost
    with opener.open(req, timeout=timeout) as resp:
        data = json.loads(resp.read().decode('utf-8'))
    return data['choices'][0]['message']['content'].strip(), data.get('usage', {})


def main():
    p = argparse.ArgumentParser(description='Chat con Gemma da terminale')
    p.add_argument('--api', default='http://127.0.0.1:8000/v1')
    p.add_argument('--model', default='google/gemma-4-E2B-it')
    p.add_argument('--system', default=DEFAULT_SYSTEM, help='prompt di sistema')
    p.add_argument('--system-file', help='prompt di sistema da un file di testo')
    p.add_argument('--temperature', type=float, default=0.7)
    p.add_argument('--max-tokens', type=int, default=200)
    p.add_argument('--timeout', type=float, default=120.0)
    a = p.parse_args()
    system = open(a.system_file).read().strip() if a.system_file else a.system
    temperature = a.temperature
    messages = [{'role': 'system', 'content': system}]
    print(f'Prompt di sistema: {system}\nTemperatura: {temperature}   (/help per i comandi, /quit per uscire)\n')

    while True:
        try:
            text = input('tu> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not text:
            continue
        if text in ('/quit', '/exit'):
            break
        if text == '/help':
            print(__doc__)
            continue
        if text == '/reset':
            messages = [{'role': 'system', 'content': system}]
            print('(conversazione azzerata)')
            continue
        if text.startswith('/system '):
            system = text[len('/system '):].strip()
            messages = [{'role': 'system', 'content': system}]
            print(f'(nuovo prompt di sistema, conversazione azzerata)')
            continue
        if text.startswith('/temp '):
            temperature = float(text.split()[1])
            print(f'(temperatura {temperature})')
            continue
        messages.append({'role': 'user', 'content': text})
        t0 = time.time()
        try:
            answer, usage = ask(a.api, a.model, messages, temperature, a.max_tokens, a.timeout)
        except Exception as e:  # noqa: BLE001
            messages.pop()
            print(f'(errore: {e} — llama.cpp è acceso? curl --noproxy "*" {a.api}/models)')
            continue
        messages.append({'role': 'assistant', 'content': answer})
        print(f'gemma> {answer}')
        print(f'       ({time.time() - t0:.1f} s, {usage.get("completion_tokens", "?")} token)\n')


if __name__ == '__main__':
    main()
