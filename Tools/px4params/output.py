class Output(object):
    def Save(self, groups, fn):
        data = self.Generate(groups)
        with open(fn, 'w') as f:
            f.write(data)
